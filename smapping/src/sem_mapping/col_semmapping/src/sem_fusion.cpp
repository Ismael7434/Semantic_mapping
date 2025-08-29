#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <col_semmapping/sem_fusion.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace semmapping
{
    SemanticFusion::SemanticFusion() {}

    void SemanticFusion::show_map_id()
    {
        ROS_INFO_STREAM("Semantic Fusion Node");
    }

    bool SemanticFusion::setPriorKnowledge(std::istream &input)
    {
        YAML::Node map = YAML::Load(input);
        for (const YAML::Node &entry : map)
        {
            CategoryPriorKnowledge newCategory;
            newCategory.name = entry["name"].as<std::string>();
            newCategory.length = entry["length"].as<double>();
            newCategory.width = entry["width"].as<double>();
            newCategory.objectNumber = entry["objectNumber"].as<int>();
            if(inConsideredObjectList(newCategory.name)){
                CategoriesPriorKnowledge.push_back(newCategory);
                cout << "name: "<<newCategory.name << " - length: " << newCategory.length << " - width: " <<newCategory.width<< " - number: " <<newCategory.objectNumber<<endl;
            }
        }
        cout << "Prior Knowledge loaded !" << endl;
        return true;
    }

    bool SemanticFusion::updatePriorKnowledge(std::istream &input)
    {
        CategoriesPriorKnowledge.clear();
        setPriorKnowledge(input);
        return true;
    }

    std::list<CategoryPriorKnowledge> SemanticFusion::getPriorKnowledge(){
        return CategoriesPriorKnowledge;
    }

    CategoryPriorKnowledge SemanticFusion::getCategoryPriorKnowledge(std::string category){
        CategoryPriorKnowledge cat;
        for(auto val : CategoriesPriorKnowledge){
            if(val.name == category)
                cat = val;
        }
        return cat;
    }

    bool SemanticFusion::initializeCategoryObjectNumber(){
        for(auto val : consideredCategories){
            std::pair<std::string, int> CategoryAndNumber = { val, 0 };
            ObjectNumberInCategory.push_back(CategoryAndNumber);
        }
        return 1;
    }

    int SemanticFusion::getCategoryObjectNumber(std::string category){
        for(auto val : ObjectNumberInCategory){
            if(val.first == category)
                return val.second;
        }
        return 0;
    }
    
    bool SemanticFusion::updateCategoryObjectNumber(std::string category, std::string incrementDecrement){
        for(auto &val : ObjectNumberInCategory){
            if(val.first == category){
                if(incrementDecrement == "increment")
                    val.second ++;
                else
                    val.second --;
                return 1;
            }
        }
        return 1;
    }
    
    void SemanticFusion::countObjectNumberPerCategoryInMap(semmapping::SemanticMap map){
        for (auto &value : map.getObjectList()){
            SemanticObject &obj = value.second;
            updateCategoryObjectNumber(obj.name, "increment");
        }
    }
    
    SemanticObject SemanticFusion::copySemanticObject(SemanticObject obj)
    {
        SemanticObject c_obj;
        c_obj.name = obj.name;
        c_obj.rotation_angle = 0;
        c_obj.isCombined = false;
        c_obj.exist_certainty = obj.exist_certainty;
        c_obj.shape_union = obj.shape_union;
        c_obj.obb = obj.obb;
        c_obj.obb_score = obj.obb_score;
        c_obj.bounding_box = bg::return_envelope<box>(obj.obb);
        bg::centroid(c_obj.shape_union, c_obj.shape_union_cen);
        bg::centroid(c_obj.obb, c_obj.oriented_box_cen);
        c_obj.point_cloud = nullptr;
        return c_obj;
    }

    point SemanticFusion::getObbLeftTop(polygon obb){
        point extreme_left_top= obb.outer()[0];
        for(int i=0; i < obb.outer().size()-1;i++){
            if(obb.outer()[i].x()<=extreme_left_top.x() && obb.outer()[i].y()>=extreme_left_top.y())
                extreme_left_top=obb.outer()[i];
        }
        return extreme_left_top;
    }

    point SemanticFusion::getObbRightTop(polygon obb){
        point extreme_right_top= obb.outer()[0];
        for(int i=0; i < obb.outer().size()-1;i++){
            if(obb.outer()[i].x()>=extreme_right_top.x() && obb.outer()[i].y()>=extreme_right_top.y())
                extreme_right_top=obb.outer()[i];
        }
        return extreme_right_top;
    }
    
    point SemanticFusion::getNearestPoint(point p, polygon poly){
        point n_point;
        double min_distance = sqrt((poly.outer()[0].x() - p.x()) * (poly.outer()[0].x() - p.x()) 
                                + (poly.outer()[0].y() - p.y()) * (poly.outer()[0].y() - p.y()));
        n_point = poly.outer()[0];
        for (int i=0; i < poly.outer().size()-1; i++){
            double distance = sqrt((poly.outer()[i].x() - p.x()) * (poly.outer()[i].x() - p.x()) 
                                + (poly.outer()[i].y() - p.y()) * (poly.outer()[i].y() - p.y()));
            if (distance < min_distance) {
                min_distance = distance;
                n_point = poly.outer()[i];
            }
        }
        return n_point;
    }

    int SemanticFusion::getNearestPointIndex(point p, polygon poly){
        double min_distance = sqrt((poly.outer()[0].x() - p.x()) * (poly.outer()[0].x() - p.x()) 
                                + (poly.outer()[0].y() - p.y()) * (poly.outer()[0].y() - p.y()));
        int n_index = 0;
        for (int i=0; i < poly.outer().size()-1; i++){
            double distance = sqrt((poly.outer()[i].x() - p.x()) * (poly.outer()[i].x() - p.x()) 
                                + (poly.outer()[i].y() - p.y()) * (poly.outer()[i].y() - p.y()));
            if (distance < min_distance) {
                min_distance = distance;
                n_index = i;
            }
        }
        return n_index;
    }
    
    polygon SemanticFusion::computeMeanBox(polygon obb1, polygon obb2){
        point obb1_cen, obb2_cen, fused_obb_cen;
        bg::centroid(obb1, obb1_cen);
        bg::centroid(obb2, obb2_cen);
        fused_obb_cen.x((obb1_cen.x() + obb2_cen.x()) / 2);
        fused_obb_cen.y((obb1_cen.y() + obb2_cen.y()) / 2);

        point origin; origin.x(0); origin.y(0);
        polygon rot_obb1, trans_obb1, trans_obb2, fused_obb;
        point obb1TranslationVector, obb2TranslationVector;
        obb1TranslationVector.x(origin.x() - obb1_cen.x());
        obb1TranslationVector.y(origin.y() - obb1_cen.y());
        obb2TranslationVector.x(origin.x() - obb2_cen.x());
        obb2TranslationVector.y(origin.y() - obb2_cen.y());
        bg::strategy::transform::translate_transformer<double, 2, 2> translateObb1ToOrigin(obb1TranslationVector.x(), obb1TranslationVector.y());
        bg::strategy::transform::translate_transformer<double, 2, 2> translateObb2ToOrigin(obb2TranslationVector.x(), obb2TranslationVector.y());
        bg::transform(obb1, trans_obb1, translateObb1ToOrigin);
        bg::transform(obb2, trans_obb2, translateObb2ToOrigin);

        point b = getObbRightTop(trans_obb1);
        point f = getNearestPoint(b, trans_obb2);
        double dot_product= (b.x() - origin.x()) * (f.x() - origin.x()) + (b.y() - origin.y()) * (f.y() - origin.y());
        double magn1=sqrt((b.y() - origin.y()) * (b.y() - origin.y()) + (b.x() - origin.x()) * (b.x() - origin.x()));
        double magn2=sqrt((f.y() - origin.y()) * (f.y() - origin.y()) + (f.x() - origin.x()) * (f.x() - origin.x()));
        double cos_angle= dot_product / (magn1 * magn2);
        double rot_angle;
        if(cos_angle>= -1 && cos_angle<=1){
            if(b.x() < f.x())
                rot_angle = std::acos(cos_angle)/2;
            else
                rot_angle = (-1) * std::acos(cos_angle)/2;
        }
        else
            rot_angle = 0.0;          

        bg::strategy::transform::matrix_transformer<double, 2, 2> rot(
            cos(rot_angle), sin(rot_angle), 0,
           -sin(rot_angle), cos(rot_angle), 0,
            0,          0,          1);
        bg::strategy::transform::translate_transformer<double, 2, 2> final_translate(fused_obb_cen.x(), fused_obb_cen.y());
        bg::transform(trans_obb1, rot_obb1, rot);
        bg::transform(rot_obb1, fused_obb, final_translate);
        return fused_obb;
    }

    polygon SemanticFusion::reorderObbVertices(polygon to_reorder, polygon to_reorder_like){
        // Find the vertice of the polygon to_reorder associated to the first point in the polygon to_reorder_like
        int index = getNearestPointIndex(to_reorder_like.outer()[0], to_reorder);
        polygon poly_reodered;
        int j=0;
        for (int i= index; i < 4; i++)
            bg::append(poly_reodered.outer(), to_reorder.outer()[i]);
        
        for (int i= 0; i <= index; i++)
            bg::append(poly_reodered.outer(), to_reorder.outer()[i]);
           
        return poly_reodered;
    }
    
    polygon SemanticFusion::ekfRectangularBoxesFusion(polygon initial_obb, polygon new_obb, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        VectorXd x(8);
        // Initialize the state vector
        x << initial_obb.outer()[0].x(), initial_obb.outer()[0].y(),
            initial_obb.outer()[1].x(), initial_obb.outer()[1].y(),
            initial_obb.outer()[2].x(), initial_obb.outer()[2].y(),
            initial_obb.outer()[3].x(), initial_obb.outer()[3].y();

        new_obb = reorderObbVertices(new_obb, initial_obb);
        VectorXd z(8);
        // Initialize the state vector and covariance matrix
        z << new_obb.outer()[0].x(), new_obb.outer()[0].y(),
            new_obb.outer()[1].x(), new_obb.outer()[1].y(),
            new_obb.outer()[2].x(), new_obb.outer()[2].y(),
            new_obb.outer()[3].x(), new_obb.outer()[3].y();

        // Theorotical and meserment models
        MatrixXd A = MatrixXd::Identity(8, 8);  
        MatrixXd H = MatrixXd::Identity(8, 8);

        // Prediction step
        x = A * x;
        P = A * P * A.transpose() + Q;

        // Update step
        VectorXd y = z - H * x;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
        x = x + K * y;
        P = (MatrixXd::Identity(8, 8) - K * H) * P;

        polygon fused_obb;
        point p1, p2, p3, p4;
        p1.x(x(0)); p1.y(x(1));
        p2.x(x(2)); p2.y(x(3));
        p3.x(x(4)); p3.y(x(5));
        p4.x(x(6)); p4.y(x(7));

        bg::append(fused_obb.outer(), p1);
        bg::append(fused_obb.outer(), p2);
        bg::append(fused_obb.outer(), p3);
        bg::append(fused_obb.outer(), p4);
        bg::append(fused_obb.outer(), p1);

        //cout << "P= " << P(0,0) << ", " << P(1,1) << ", " << P(2,2) << ", " << P(3,3) << ", ... , " << P(7,7) << endl;
        //ROS_INFO_STREAM("FINISHED EKF PROCESS");
        
        return fused_obb;
    }

    // Semantic Object fusion function: returns object OBB with the best score
    SemanticObject SemanticFusion::nmsFusionOfSemanticObject(SemanticObject initial_obj, SemanticObject received_obj)
    {
        SemanticObject obj;
        if (initial_obj.obb_score > received_obj.obb_score)
            obj = copySemanticObject(initial_obj);
        else
            obj = copySemanticObject(received_obj);

        //Partial polygons of fused object are stored in obj.shapes
        point initial_obj_centroid, received_obj_centroid;
        bg::centroid(initial_obj.shape_union, initial_obj_centroid); bg::centroid(received_obj.shape_union, received_obj_centroid);
        obj.shapes.push_back({initial_obj.shape_union, initial_obj_centroid, initial_obj.exist_certainty});
        obj.shapes.push_back({received_obj.shape_union, received_obj_centroid, received_obj.exist_certainty});
        obj.isCombined = true;
        return obj;
    }

    // Calculate the oriented bounding box (OBB) score between two shapes.
    double SemanticFusion::calculateOBBscore(const SemanticObject& obj1, const SemanticObject& obj2) {
        return (ref_fit(obj1.obb, obj2.shape_union) + ref_fit(obj1.obb, obj1.shape_union)) / 2;
    }
    
    // Perform geometric fusion for two rectangular objects.
    SemanticObject SemanticFusion::objectWithObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj) {
        double initial_obb_score = calculateOBBscore(initial_obj, received_obj);
        double received_obb_score = calculateOBBscore(received_obj, initial_obj);    

        SemanticObject obj;
        if (iou(initial_obj.obb, received_obj.obb) < 0.6) {
            if (initial_obb_score > received_obb_score)
                obj = initial_obj;
            else
                obj = received_obj;
        } else {
            obj = initial_obj;
            obj.obb = computeMeanBox(initial_obj.obb, received_obj.obb);
            obj.shape_union = obj.obb;
            obj.obb_score = (initial_obj.obb_score + received_obj.obb_score) / 2;
            obj.bounding_box = bg::return_envelope<box>(obj.obb);
            bg::centroid(obj.shape_union, obj.shape_union_cen);
            bg::centroid(obj.obb, obj.oriented_box_cen);
            obj.point_cloud = nullptr;
        }

        obj.isCombined = true;
        obj.exist_certainty = (initial_obj.exist_certainty + received_obj.exist_certainty) / 2;
        return obj;
    }

    // Perform geometric fusion for two rectangular objects.
    SemanticObject SemanticFusion::objectWithObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj, MatrixXd& P, MatrixXd& Q, MatrixXd& R) {
        double initial_obb_score = calculateOBBscore(initial_obj, received_obj);
        double received_obb_score = calculateOBBscore(received_obj, initial_obj);    

        SemanticObject obj;
        if (iou(initial_obj.obb, received_obj.obb) < 0.6) { //0.7
            if (initial_obb_score > received_obb_score)
                obj = initial_obj;
            else
                obj = received_obj;
        } 
        else {
            obj = initial_obj;
            obj.obb = ekfRectangularBoxesFusion(initial_obj.obb, received_obj.obb, P, Q, R);
            obj.shape_union = obj.obb;
            obj.obb_score = (initial_obj.obb_score + received_obj.obb_score) / 2;
            obj.bounding_box = bg::return_envelope<box>(obj.obb);
            bg::centroid(obj.shape_union, obj.shape_union_cen);
            bg::centroid(obj.obb, obj.oriented_box_cen);
            obj.point_cloud = nullptr;
        }

        obj.exist_certainty = (initial_obj.exist_certainty + received_obj.exist_certainty) / 2;
        obj.isCombined = true;
        return obj;
    }

    // Perform geometric fusion for two non-rectangular objects.
    SemanticObject SemanticFusion::objectWithoutObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj, semmapping::SemanticMap map) {
        SemanticObject obj;
        obj.name = initial_obj.name;
        obj.rotation_angle = 0;
        multi_polygon sect;
        bg::intersection(initial_obj.obb, received_obj.obb, sect);
        obj.exist_certainty = (initial_obj.exist_certainty + received_obj.exist_certainty) / 2;
        obj.shape_union = sect[0];

        // Regenerate a new OBB representation for the resulting polygon
        std::pair<polygon, double> selected_obb = map.create_object_box_using_prior_knowledge(obj.shape_union, obj.name, false); //use_first_plan_edges=false
        obj.obb = selected_obb.first;
        obj.obb_score = 1 - selected_obb.second;
        obj.bounding_box = bg::return_envelope<box>(obj.obb);
        bg::centroid(obj.shape_union, obj.shape_union_cen);
        bg::centroid(obj.obb, obj.oriented_box_cen);
        obj.point_cloud = nullptr;
        obj.isCombined = true;
        return obj;
    }

    // Main function for geometric fusion of semantic objects
    SemanticObject SemanticFusion::objectUpdate(SemanticObject initial_obj, SemanticObject received_obj, semmapping::SemanticMap map) {
        SemanticObject updatedObj;
        if (isRectangular(initial_obj.obb) && isRectangular(received_obj.obb)) {
            updatedObj = objectWithObbUpdate(initial_obj, received_obj);
        } else if (isRectangular(initial_obj.obb)) {
            updatedObj = initial_obj;
        } else if (isRectangular(received_obj.obb)) {
            updatedObj = received_obj;
        } else {
            updatedObj = objectWithoutObbUpdate(initial_obj, received_obj, map);
        }
        updatedObj.exist_certainty = (initial_obj.exist_certainty + received_obj.exist_certainty) / 2;
        updatedObj.isCombined = true;
        return updatedObj;
    }

    SemanticObject SemanticFusion::objectUpdate(SemanticObject initial_obj, SemanticObject received_obj, semmapping::SemanticMap map, MatrixXd& P, MatrixXd& Q, MatrixXd& R) {
        SemanticObject updatedObj;
        if (isRectangular(initial_obj.obb) && isRectangular(received_obj.obb)) {
            updatedObj = objectWithObbUpdate(initial_obj, received_obj, P, Q, R);
        } else if (isRectangular(initial_obj.obb)) {
            updatedObj = copySemanticObject(initial_obj);
        } else if (isRectangular(received_obj.obb)) {
            updatedObj = copySemanticObject(received_obj);
        } else {
            updatedObj = objectWithoutObbUpdate(initial_obj, received_obj, map);
        }
        updatedObj.exist_certainty = (initial_obj.exist_certainty + received_obj.exist_certainty) / 2;
        updatedObj.isCombined = true;
        return updatedObj;
    }
    
    bool SemanticFusion::similarClasses(std::string object1_name, std::string object2_name)
    {
        if ((object1_name == "Sofa bed" && object2_name == "Couch") || (object1_name == "Couch" && object2_name == "Sofa bed"))
            return true;
        else
            return false;
    }

    bool SemanticFusion::checkTheAbilityOfObjectsToOverlap(std::string object1_name, std::string object2_name){
        /*if(object1_name == object2_name)
            return true;*/
        if(similarClasses(object1_name, object2_name))
            return true;
        else if ((object1_name == "Chair" && object2_name == "Table") || (object1_name == "Table" && object2_name == "Chair"))
            return true;
        else
            return false;
    }

    bool SemanticFusion::inConsideredObjectList(std::string object_name){
        bool found = (std::find(consideredCategories.begin(), consideredCategories.end(), object_name) != consideredCategories.end());
        return found;
    }

    bool SemanticFusion::validObjectSurface(std::string object_name, polygon object_obb)
    {
        std::pair<double, double> dimensions= get_real_object_length_width(object_name);
        double area_fit= bg::area(object_obb)/(dimensions.first*dimensions.second);
        if(area_fit < 1.5){
            cout << "valid shape !" << endl;
            return 1;
        }
        else{
            cout << "not valid shape !" << endl;
            return 0;
        }
    }

    bool SemanticFusion::isRectangular(polygon object_obb)
    {
        if(object_obb.outer().size() != 5)
            return false;
        //When a polygon has 4 edges, check if it is a rectangle    
        //Check the length of the diagonals, the diagonals must be equal.
        double diag1_distance= sqrt((object_obb.outer()[2].x() - object_obb.outer()[0].x())*(object_obb.outer()[2].x() - object_obb.outer()[0].x()) 
                            + (object_obb.outer()[2].y() - object_obb.outer()[0].y())*(object_obb.outer()[2].y() - object_obb.outer()[0].y()));
        double diag2_distance= sqrt((object_obb.outer()[3].x() - object_obb.outer()[1].x())*(object_obb.outer()[3].x() - object_obb.outer()[1].x()) 
                            + (object_obb.outer()[3].y() - object_obb.outer()[1].y())*(object_obb.outer()[3].y() - object_obb.outer()[1].y()));
        double error = 0.02;
        bool condition1= std::abs(diag1_distance - diag2_distance) < error;
        if(!condition1)
            return false;
        //Determine polygon diagonals intersection
        double a = object_obb.outer()[2].y() - object_obb.outer()[0].y();
        double b = object_obb.outer()[0].x() - object_obb.outer()[2].x();
        double c = (a * object_obb.outer()[0].x()) + (b * object_obb.outer()[0].y());
        double a1 = object_obb.outer()[1].y() - object_obb.outer()[3].y();
        double b1 = object_obb.outer()[3].x() - object_obb.outer()[1].x();
        double c1 = (a1 * object_obb.outer()[3].x()) + (b1 * object_obb.outer()[3].y());
        double det = a*b1 - a1*b;
        double x_intersection = (b1*c - b*c1)/det;
        double y_intersection = (a*c1 - a1*c)/det;
        //The polygon is rectongular if diagonals are equal length and bisect
        bool condition2= std::abs(x_intersection - ((object_obb.outer()[0].x() + object_obb.outer()[2].x())/2)) < error
                        && std::abs(y_intersection - ((object_obb.outer()[0].y() + object_obb.outer()[2].y())/2)) < error;
        bool condition3= std::abs(x_intersection - ((object_obb.outer()[1].x() + object_obb.outer()[3].x())/2)) < error
                        && std::abs(y_intersection - ((object_obb.outer()[1].y() + object_obb.outer()[3].y())/2)) < error;
        if(condition2 && condition3)
            return true;
        else
            return false;
    }
    
    void SemanticFusion::removeMapInconsistencies(semmapping::SemanticMap map, semmapping::SemanticMap &filtered_map, double overlap_threshold)
    {
        if (map.getObjectList().empty())
            cout<< "Map is empty, no filtering needed." <<endl;

        filtered_map.clearAll();

        for (auto &val : map.getObjectList()){
            SemanticObject &new_obj = val.second;

            if (new_obj.exist_certainty <= 0.25 || !inConsideredObjectList(new_obj.name)){
                continue; // Skip objects with low certainty or not in the considered list
            }
            
            bool addedToFilteredMap = false;
            bool isOverlapping = false;

            for (auto &val2 : filtered_map.getObjectList()){
                SemanticObject &ref_obj = val2.second;
                double overlap = iou(ref_obj.obb, new_obj.obb);
                
                if((new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name)) && overlap > overlap_threshold){
                    addedToFilteredMap = true;
                    isOverlapping = true;
                    SemanticObject fused_obj = objectUpdate(ref_obj, new_obj, filtered_map);
                    filtered_map.removeObject(val2.first);
                    filtered_map.addObject(fused_obj);
                    break; // Exit the loop as we have already added the fused object
                }   
            }

            if(!addedToFilteredMap){
                for (auto &val2 : filtered_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);
                    bool possible_overlap = checkTheAbilityOfObjectsToOverlap(new_obj.name, ref_obj.name);

                    if(overlap > overlap_threshold && !addedToFilteredMap){
                        isOverlapping = true;

                        if(possible_overlap){ //Overlap between objects is possible
                            if (!(new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name))){
                                new_obj.point_cloud = nullptr;
                                filtered_map.addObject(new_obj);
                                addedToFilteredMap = true;
                                break; // Exit the loop as we have already added the fused object
                            }
                        }
                        else //Overlap between objects is not possible 
                        {
                            bool shape_obj1_is_valid = validObjectSurface(new_obj.name, new_obj.obb);
                            bool shape_obj2_is_valid = validObjectSurface(ref_obj.name, ref_obj.obb);
                            if(shape_obj1_is_valid && (!shape_obj2_is_valid || (bg::area(new_obj.obb) > bg::area(ref_obj.obb)))){
                                filtered_map.removeObject(val2.first);
                                new_obj.point_cloud = nullptr;
                                filtered_map.addObject(new_obj);
                                addedToFilteredMap = true;
                                break; // Exit the loop as we have already added the fused object
                            }
                        }
                    }   
                }
            }
            if(!isOverlapping && !addedToFilteredMap){ //Add objects that do not overlap with other objects to the map
                new_obj.point_cloud = nullptr;
                filtered_map.addObject(new_obj);
            }
            
        }
    }

    void SemanticFusion::semfusion(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold, std::string algorithm){
        if (received_map.getObjectList().empty()){
            cout << "Received Map is empty!" << endl;
            global_map.setObjectList(reference_map.getObjectList());
        }
        if (reference_map.getObjectList().empty()){
            cout << "Global map is empty, received map is considered as global map!" << endl;
            global_map.setObjectList(received_map.getObjectList());
        }
        for (auto &val : received_map.getObjectList()){
            SemanticObject &new_obj = val.second;
            bool addedToGlobalMap = false;
            bool isOverlapping = false;
            if (new_obj.exist_certainty > 0.25){
                // Object is already in global map: update object in global map
                for (auto &val2 : global_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);
                    if(overlap > overlap_threshold && (new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name))){
                        isOverlapping = true; //parameter indicating that the object's polygon overlaps another instance of the object or another object.
                        SemanticObject fused_obj;
                        if(algorithm == "our_solution")
                            fused_obj = objectUpdate(ref_obj, new_obj, global_map);
                        else
                            fused_obj= nmsFusionOfSemanticObject(ref_obj, new_obj);
                        global_map.removeObject(val2.first);
                        global_map.addObject(fused_obj);
                        addedToGlobalMap = true; //parameter used to determine whether or not the object is already in the global map
                    }   
                }
                // Object is not in the global map: Add fused object from both maps to global map
                if(!addedToGlobalMap){
                    for (auto &val2 : reference_map.getObjectList()){
                        SemanticObject &ref_obj = val2.second;
                        double overlap = iou(new_obj.obb, ref_obj.obb);
                        if (overlap > overlap_threshold && (new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name))){
                            isOverlapping = true;
                            SemanticObject fused_obj;
                            if(algorithm == "our_solution")
                                fused_obj = objectUpdate(ref_obj, new_obj, global_map);
                            else
                                fused_obj= nmsFusionOfSemanticObject(ref_obj, new_obj);
                            global_map.addObject(fused_obj);
                        }
                    }
                }
                // Add objects existing only in the received map with a certainty of existence > 0.5
                if (!isOverlapping && new_obj.exist_certainty > 0.5){
                    new_obj.point_cloud = nullptr;
                    global_map.addObject(new_obj);
                }
            }
        }
    }

    void SemanticFusion::semfusion_updated(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold){
        global_map.clearAll();
        for (auto &value : reference_map.getObjectList()){
            SemanticObject &obj = value.second;
            global_map.addObject(obj);
        }

        for (auto &val : received_map.getObjectList()){
            SemanticObject &new_obj = val.second;
            bool addedToGlobalMap = false;
            bool isOverlapping = false;
            if (new_obj.exist_certainty > 0.25){
                // Object is already in global map: update object in global map
                for (auto &val2 : global_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);
                    if(overlap && (new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name)))
                        isOverlapping = true; //parameter indicating that the object's polygon overlaps another instance of the object or another object.
                    if(overlap > overlap_threshold && (new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name))){
                        SemanticObject fused_obj;
                        fused_obj = objectUpdate(ref_obj, new_obj, global_map);
                        global_map.removeObject(val2.first);
                        global_map.addObject(fused_obj);
                        addedToGlobalMap = true; //parameter used to determine whether or not the object is already in the global map
                    }   
                }
                // Add objects existing only in the received map with a certainty of existence > 0.5
                if (!isOverlapping && !addedToGlobalMap && new_obj.exist_certainty > 0.5 && new_obj.obb_score > 0){
                    new_obj.point_cloud = nullptr;
                    global_map.addObject(new_obj);
                }
            }
        }
    }
    
    void SemanticFusion::globalMapUpdate(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, semmapping::SemanticMap &waiting_objects,
        MatrixXd& P, MatrixXd& Q, MatrixXd& R, double overlap_threshold){
        global_map.clearAll();
        
        // Initiliaze the global map with objects of the reference map (isCombined must be initialized to false)
        for (auto &value : reference_map.getObjectList()){
            SemanticObject &obj = value.second;
            obj.isCombined = false;
            global_map.addObject(obj);
        }

        // This step initializes the parameter isCombined to False for objects in the waiting list
        // This parameter will be changed to True for re-observed Objects (used to reduce existence probability for un-observed objects)
        for (auto &wObj2 : waiting_objects.getObjectList()){
            SemanticObject &w_obj = wObj2.second;
            w_obj.isCombined = false;
            waiting_objects.removeObject(wObj2.first);
            waiting_objects.addObject(w_obj);
        }

        // Global map & waiting list updating process
        for (auto &val : received_map.getObjectList()){
            SemanticObject &new_obj = val.second;
            bool addedToGlobalMap = false;
            bool inWaitingObjects = false;

            if (new_obj.exist_certainty > 0.25){
                // Updating objects already in the global map
                for (auto &val2 : global_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);
                    // Ignore new detections of objects existing in the map with overlap < overlap_threshold
                    if(overlap && (new_obj.name == ref_obj.name || similarClasses(new_obj.name, ref_obj.name))){
                        addedToGlobalMap = true; 
                        if(overlap > overlap_threshold){
                            SemanticObject fused_obj = objectUpdate(ref_obj, new_obj, global_map, P, Q, R);
                            global_map.removeObject(val2.first);
                            global_map.addObject(fused_obj);
                        }   
                        break; // No need to continue iterating once the object is added
                    }
                }

                if (!addedToGlobalMap){
                    for (auto &wObj : waiting_objects.getObjectList()){
                        SemanticObject &wRefObj = wObj.second;
                        double overlap = iou(wRefObj.obb, new_obj.obb);

                        if(overlap && (new_obj.name == wRefObj.name || similarClasses(new_obj.name, wRefObj.name))){
                            inWaitingObjects = true; 
                            if(overlap > overlap_threshold){
                                SemanticObject fused_obj = objectUpdate(wRefObj, new_obj, waiting_objects, P, Q, R);
                                waiting_objects.removeObject(wObj.first);
                                waiting_objects.addObject(fused_obj);
                            }   
                            break; // No need to continue iterating once the object is added
                        }
                    }
                }

                // Adding new object when an OBB is created and the number of mapped objects from that category is still less then the category total number of objects
                // Otherwise, adding the object to the waiting list
                if (!addedToGlobalMap && !inWaitingObjects && new_obj.obb_score > 0){
                    CategoryPriorKnowledge classKnowledge = getCategoryPriorKnowledge(new_obj.name);
                    if(getCategoryObjectNumber(new_obj.name)< classKnowledge.objectNumber){
                        new_obj.point_cloud = nullptr;
                        new_obj.isCombined = true;
                        global_map.addObject(new_obj);
                        updateCategoryObjectNumber(new_obj.name, "increment");
                    }
                    else
                    {
                        /*bool inWaitingObjects = false;
                        for (auto &wObj : waiting_objects.getObjectList()){
                            SemanticObject &wRefObj = wObj.second;
                            double overlap = iou(wRefObj.obb, new_obj.obb);

                            if(overlap && (new_obj.name == wRefObj.name || similarClasses(new_obj.name, wRefObj.name))){
                                inWaitingObjects = true; 
                                if(overlap > overlap_threshold){
                                    SemanticObject fused_obj = objectUpdate(wRefObj, new_obj, waiting_objects, P, Q, R);
                                    waiting_objects.removeObject(wObj.first);
                                    waiting_objects.addObject(fused_obj);
                                }   
                            }
                        }*/

                        new_obj.point_cloud = nullptr;
                        new_obj.isCombined = true;
                        waiting_objects.addObject(new_obj);
                    }
                }
            }
        }

        // Update existence probability for objects not re-observed in the main map
        for (auto &val3 : global_map.getObjectList()){
            SemanticObject &map_obj = val3.second;
            if(!map_obj.isCombined){
                map_obj.exist_certainty = map_obj.exist_certainty - 0.25;
                global_map.removeObject(val3.first);

                if(map_obj.exist_certainty >= 0.25)
                    global_map.addObject(map_obj);
                else
                    updateCategoryObjectNumber(map_obj.name, "decrement");
            }
        }

        // Update existence probability for objects not re-observed in the waiting list
        for (auto &wObj2 : waiting_objects.getObjectList()){
            SemanticObject &waiting_obj = wObj2.second;
            if(!waiting_obj.isCombined){
                waiting_obj.exist_certainty = waiting_obj.exist_certainty - 0.25;
                waiting_objects.removeObject(wObj2.first);
                if(waiting_obj.exist_certainty >= 0.25)
                    waiting_objects.addObject(waiting_obj);
            }
        }

        // Move objects from waiting list to the main map if valid condition
        for(const std::string category: consideredCategories){
            CategoryPriorKnowledge classKnowledge = getCategoryPriorKnowledge(category);
            for (auto &wObj2 : waiting_objects.getObjectList()){
                SemanticObject &waiting_obj = wObj2.second;
                if(waiting_obj.name == category && getCategoryObjectNumber(waiting_obj.name)< classKnowledge.objectNumber){
                    waiting_objects.removeObject(wObj2.first);
                    global_map.addObject(waiting_obj);
                    updateCategoryObjectNumber(waiting_obj.name, "increment");
                }
            }
        }
    }

    void SemanticFusion::semfusion_modified_nms(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold){
        global_map.clearAll();
        for (auto &value : reference_map.getObjectList()){
            SemanticObject &obj = value.second;
            global_map.addObject(obj);
        }

        for (auto &val : received_map.getObjectList()){
            SemanticObject &new_obj = val.second;
            bool addedToGlobalMap = false;
            bool isOverlapping = false;
            if (new_obj.exist_certainty > 0.25){
                // Object is already in global map: update object in global map
                for (auto &val2 : global_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);
                    if(overlap && new_obj.name == ref_obj.name)
                        isOverlapping = true; //parameter indicating that the object's polygon overlaps another instance of the object or another object.
                    if(overlap > overlap_threshold && new_obj.name == ref_obj.name){
                        SemanticObject fused_obj;
                        fused_obj= nmsFusionOfSemanticObject(ref_obj, new_obj);
                        global_map.removeObject(val2.first);
                        global_map.addObject(fused_obj);
                        addedToGlobalMap = true; //parameter used to determine whether or not the object is already in the global map
                    }   
                }
                // Add objects existing only in the received map with a certainty of existence > 0.5
                if (!isOverlapping && !addedToGlobalMap && new_obj.exist_certainty > 0.5 && new_obj.obb_score > 0){
                    new_obj.point_cloud = nullptr;
                    global_map.addObject(new_obj);
                }
            }
        }
    }

    void SemanticFusion::semfusion_nms(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold){
        global_map.clearAll();
        for (auto &value : reference_map.getObjectList()){
            SemanticObject &obj = value.second;
            global_map.addObject(obj);
        }

        for (auto &val : received_map.getObjectList()){
            SemanticObject &new_obj = val.second;
            if (new_obj.exist_certainty > 0.25){
                // Object is already in global map: update object in global map
                for (auto &val2 : global_map.getObjectList()){
                    SemanticObject &ref_obj = val2.second;
                    double overlap = iou(ref_obj.obb, new_obj.obb);

                    if(overlap > overlap_threshold && new_obj.name == ref_obj.name){
                        SemanticObject fused_obj;
                        fused_obj= nmsFusionOfSemanticObject(ref_obj, new_obj);
                        global_map.removeObject(val2.first);
                        global_map.addObject(fused_obj);
                        break;
                    }   
                }
            }
        }
    }

    // Methods for evaluating merged maps

    bool SemanticFusion::isCloseDistance(double distance1, double distance2){
        if(abs(distance1-distance2) < 0.1)
            return true;
        else
            return false;
    }

    double SemanticFusion::objectOrientation(polygon poly){
        double edge1_distance = sqrt((poly.outer()[1].x() - poly.outer()[0].x())*(poly.outer()[1].x() - poly.outer()[0].x()) 
                                + (poly.outer()[1].y() - poly.outer()[0].y())*(poly.outer()[1].y() - poly.outer()[0].y()));

        double edge2_distance = sqrt((poly.outer()[2].x() - poly.outer()[1].x())*(poly.outer()[2].x() - poly.outer()[1].x()) 
                                + (poly.outer()[2].y() - poly.outer()[1].y())*(poly.outer()[2].y() - poly.outer()[1].y()));

        double angle, angle_degrees;
        if(edge1_distance > edge2_distance)
            angle = atan2(poly.outer()[1].y() - poly.outer()[0].y(), poly.outer()[1].x() - poly.outer()[0].x());
        else
            angle = atan2(poly.outer()[2].y() - poly.outer()[1].y(), poly.outer()[2].x() - poly.outer()[1].x());

        angle_degrees = angle * (180/ M_PI);
        
        if (angle_degrees < 0)
            angle_degrees += 180;
        else if (angle_degrees > 180)
            angle_degrees -= 180;

        return angle_degrees;
    }

    void SemanticFusion::updateClassStats(ClassStats& stats, double overlap, double comOffset, double orientationOffset) {
        stats.true_positive++;
        stats.mean_iou = (stats.mean_iou * (stats.true_positive - 1) + overlap) / stats.true_positive;
        stats.mean_com_offset = (stats.mean_com_offset * (stats.true_positive - 1) + comOffset) / stats.true_positive;
        stats.mean_orientation_offset = (stats.mean_orientation_offset * (stats.true_positive - 1) + orientationOffset) / stats.true_positive;
    }

    void SemanticFusion::saveMapStats(const std::map<std::string, ClassStats>& all_classes_stats, const std::string& filename) {
        std::ofstream myfile;
        myfile.open(filename, std::ios::out | std::ios::app);
        myfile << "Map average metrics" << "\n";
        
        for (const auto& classData : all_classes_stats) {
            const std::string& className = classData.first;
            const ClassStats& stats = classData.second;
            myfile << className << "," << stats.true_positive << "," << stats.false_positive << "," << stats.false_negative << "," << stats.mean_iou << "," << stats.mean_com_offset << "," << stats.mean_orientation_offset << "," << stats.F1_score << "\n";
        }
        
        myfile.close();
    }

    void SemanticFusion::saveObjectStats(int id, std::string category, double overlap, double ComError, double orientationError, const std::string& filename) {
        std::ofstream myfile;
        myfile.open(filename, std::ios::out | std::ios::app);
        myfile << "Object" << id << "," << category << "," << overlap << "," << ComError << "," << orientationError << "\n";
        myfile.close();
    }

    void SemanticFusion::evaluateSemanticMap(const std::map<size_t, SemanticObject>& objectList, const std::map<size_t, SemanticObject>& groundTruthObjectList, const std::string& backup_file_name) {

        std::map<std::string, ClassStats> all_classes_stats;
        // Populate all_classes_stats with class names and initialize to zero
        all_classes_stats["Chair"] = ClassStats();
        all_classes_stats["Table"] = ClassStats();
        all_classes_stats["Shelf"] = ClassStats();
        all_classes_stats["Couch"] = ClassStats();
        int numberObjectsWithOBB = 0;

        if (!objectList.empty()) {
            for (const auto& val : objectList) {
                const SemanticObject& obj = val.second;
                bool object_not_in_ground_truth_map = true;

                if (obj.exist_certainty > 0.25) {
                    for (const auto& val2 : groundTruthObjectList) {
                        const SemanticObject& gtObj = val2.second;
                        if (obj.name == gtObj.name || similarClasses(obj.name, gtObj.name)) {
                            point truth_centroid; bg::centroid(gtObj.obb, truth_centroid);
                            double com_offset = bg::distance(obj.oriented_box_cen, truth_centroid);
                            
                            // Used to verify overlap
                            double overlap_with_polygon= iou(obj.shape_union, gtObj.obb);

                            // Compute the mean overlap indicator
                            double overlap= iou(obj.obb, gtObj.obb);

                            // Verify that object OBB is rectangular
                            bool obbIsRectangular = isRectangular(obj.obb);

                            // Compute the orientation offset indicator if the OBB is rectangular
                            double orientation_offset;
                            if (obbIsRectangular){
                                orientation_offset= abs(objectOrientation(gtObj.obb)-objectOrientation(obj.obb));
                                std::pair<double, double> dimensions= get_real_object_length_width(obj.name);

                                if (isCloseDistance(dimensions.first, dimensions.second) && orientation_offset > 60)
                                    orientation_offset = abs(orientation_offset-90);
                                if (orientation_offset > 170)
                                    orientation_offset = abs(orientation_offset-180);
                            }

                            if (overlap_with_polygon && obbIsRectangular){
                                std::cout<<"obj "<< obj.name << " gt orientation = "<<objectOrientation(gtObj.obb)<<" | obj orientation = "<<objectOrientation(obj.obb)
                                    <<" | orientation error = "<< orientation_offset << endl;
                                numberObjectsWithOBB ++;

                                //saveObjectStats(numberObjectsWithOBB, obj.name, overlap, com_offset, orientation_offset, backup_file_name);

                                object_not_in_ground_truth_map = false;
                                auto it = all_classes_stats.find(obj.name);
                                if (it != all_classes_stats.end()) {
                                    updateClassStats(it->second, overlap, com_offset, orientation_offset);
                                }
                            } else if(overlap_with_polygon){
                                std::cout<<"obj "<< obj.name << " is not rectangular, considered in number of f1 score but not the other metrics."<< endl;
                                object_not_in_ground_truth_map = false;
                                auto it = all_classes_stats.find(obj.name);
                                if (it != all_classes_stats.end())
                                    it->second.true_positive++;
                            }
                        }
                    }

                    if (object_not_in_ground_truth_map) {
                        auto it = all_classes_stats.find(obj.name);
                        if (it != all_classes_stats.end())
                            it->second.false_positive++;
                    }

                    CategoryPriorKnowledge classKnowledge = getCategoryPriorKnowledge(obj.name);
                    auto it = all_classes_stats.find(obj.name);
                        if (it != all_classes_stats.end()){
                            it->second.false_negative= classKnowledge.objectNumber - it->second.true_positive;
                            std::array<double,3> F1_score_metrics;
                            F1_score_metrics = computeMapF1Score(it->second.true_positive, it->second.false_positive, it->second.false_negative);
                            it->second.F1_score= F1_score_metrics[2];
                        }
                }
            }

            // ... Output and save statistics ...

            std::cout << "--- Map Evaluation ---" << std::endl;
            std::cout << std::left << std::setw(20) << "class name" << std::setw(20) << "TP" << std::setw(20) << "FP" << std::setw(30) << "FN - Missing objects " << std::setw(20) << "Mean IOU" 
            << std::setw(20) << "Mean CoM offset" << std::setw(40) << "Mean orientation offset"<< std::setw(20) << "F1 score" << std::endl;

            for (const auto& classData : all_classes_stats) {
                const ClassStats& stats = classData.second;
                std::cout << std::left << std::setw(20) << classData.first << std::setw(20) << stats.true_positive << std::setw(20) << stats.false_positive << std::setw(30) << stats.false_negative
                    << std::setw(20) << stats.mean_iou << std::setw(20) << stats.mean_com_offset << std::setw(40) << stats.mean_orientation_offset << std::setw(20) << stats.F1_score << std::endl;
            }
            //saveMapStats(all_classes_stats, backup_file_name);

        } else {
            ROS_INFO_STREAM("Map is empty, so it can't be evaluated!");
        }
    }

    void SemanticFusion::evaluateObjectCategoriesOverAllMaps(int mapID, std::map<std::string, ClassStats>& all_classes_stats, int& numberObjectsWithOBB, const std::map<size_t, SemanticObject>& objectList,
        const std::map<size_t, SemanticObject>& groundTruthObjectList, const std::string& backup_file_name) {
        if (!objectList.empty()) {
            for (const auto& val : objectList) {
                const SemanticObject& obj = val.second;
                bool object_not_in_ground_truth_map = true;

                if (obj.exist_certainty > 0.25) {
                    for (const auto& val2 : groundTruthObjectList) {
                        const SemanticObject& gtObj = val2.second;
                        if (obj.name == gtObj.name || similarClasses(obj.name, gtObj.name)) {
                            point truth_centroid; bg::centroid(gtObj.obb, truth_centroid);
                            double com_offset = bg::distance(obj.oriented_box_cen, truth_centroid);
                            
                            // Used to verify overlap
                            double overlap_with_polygon= iou(obj.shape_union, gtObj.obb);

                            // Compute the mean overlap indicator
                            double overlap= iou(obj.obb, gtObj.obb);

                            // Verify that object OBB is rectangular
                            bool obbIsRectangular = isRectangular(obj.obb);

                            // Compute the orientation offset indicator if the OBB is rectangular
                            double orientation_offset;
                            if (obbIsRectangular){
                                orientation_offset= abs(objectOrientation(gtObj.obb)-objectOrientation(obj.obb));
                                std::pair<double, double> dimensions= get_real_object_length_width(obj.name);

                                if (isCloseDistance(dimensions.first, dimensions.second) && orientation_offset > 60)
                                    orientation_offset = abs(orientation_offset-90);
                                if (orientation_offset > 170)
                                    orientation_offset = abs(orientation_offset-180);
                            }

                            if (overlap_with_polygon && obbIsRectangular){
                                std::cout<<"obj "<< obj.name << " gt orientation = "<<objectOrientation(gtObj.obb)<<" | obj orientation = "<<objectOrientation(obj.obb)
                                    <<" | orientation error = "<< orientation_offset << endl;
                                numberObjectsWithOBB ++;

                                saveObjectStats(numberObjectsWithOBB, obj.name, overlap, com_offset, orientation_offset, backup_file_name);

                                object_not_in_ground_truth_map = false;
                                auto it = all_classes_stats.find(obj.name);
                                if (it != all_classes_stats.end()) {
                                    updateClassStats(it->second, overlap, com_offset, orientation_offset);
                                }
                            } else if(overlap_with_polygon){
                                std::cout<<"obj "<< obj.name << " is not rectangular, considered in number of f1 score but not the other metrics."<< endl;
                                object_not_in_ground_truth_map = false;
                                auto it = all_classes_stats.find(obj.name);
                                if (it != all_classes_stats.end())
                                    it->second.true_positive++;
                            }
                        }
                    }

                    if (object_not_in_ground_truth_map) {
                        auto it = all_classes_stats.find(obj.name);
                        if (it != all_classes_stats.end())
                            it->second.false_positive++;
                    }

                    CategoryPriorKnowledge classKnowledge = getCategoryPriorKnowledge(obj.name);
                    auto it = all_classes_stats.find(obj.name);
                        if (it != all_classes_stats.end()){
                            it->second.false_negative= (classKnowledge.objectNumber * mapID) - it->second.true_positive;
                            std::array<double,3> F1_score_metrics;
                            F1_score_metrics = computeMapF1Score(it->second.true_positive, it->second.false_positive, it->second.false_negative);
                            it->second.F1_score= F1_score_metrics[2];
                        }
                }
            }

            // ... Output and save statistics ...

            std::cout << "--- Map Evaluation ---" << std::endl;
            std::cout << std::left << std::setw(20) << "class name" << std::setw(20) << "TP" << std::setw(20) << "FP" << std::setw(30) << "FN - Missing objects " << std::setw(20) << "Mean IOU" 
            << std::setw(20) << "Mean CoM offset" << std::setw(40) << "Mean orientation offset"<< std::setw(20) << "F1 score" << std::endl;

            for (const auto& classData : all_classes_stats) {
                const ClassStats& stats = classData.second;
                std::cout << std::left << std::setw(20) << classData.first << std::setw(20) << stats.true_positive << std::setw(20) << stats.false_positive << std::setw(30) << stats.false_negative
                    << std::setw(20) << stats.mean_iou << std::setw(20) << stats.mean_com_offset << std::setw(40) << stats.mean_orientation_offset << std::setw(20) << stats.F1_score << std::endl;
            }

        } else {
            ROS_INFO_STREAM("Map is empty, so it can't be evaluated!");
        }
    }

    void SemanticFusion::trackGtObjectState(std::string objectName, int objectId, std::map<size_t, SemanticObject> groundTruthObjectList, std::map<size_t, SemanticObject> recievedObjectList, 
    std::map<size_t, SemanticObject> ObjectList, std::map<size_t, SemanticObject> waitingObjectList, ObjectState& state, std::string trackBackupFileName) {

        SemanticObject gtObj;
        for (const auto& val : groundTruthObjectList) {
            if(val.second.name == objectName && val.second.id == objectId)
                gtObj = val.second;
        }

        bool previousIsInGlobalMap= state.isInGlobalMap;
        bool previousIsInWaitingList= state.isInWaitingList;
        bool previousNotInMap= state.notInMap;
        
        state.wasMapped = false;

        for (const auto& val : recievedObjectList) {
            const SemanticObject& obj = val.second;
            if (obj.exist_certainty > 0.25 && (obj.name == gtObj.name || similarClasses(obj.name, gtObj.name))) {
                double overlap= iou(obj.shape_union, gtObj.obb);
                if (overlap){
                    state.wasMapped = true;
                    std::cout<< gtObj.name <<" is mapped."<< endl;
                }
            }
        }

        if(!state.wasMapped)
            std::cout << gtObj.name << " is not mapped."<< endl;

        int foundInGlobalMap = 0;
        int foundInWaitingList = 0;
        for (const auto& val : ObjectList) {
            const SemanticObject& obj = val.second;
            if (obj.exist_certainty > 0.25 && (obj.name == gtObj.name || similarClasses(obj.name, gtObj.name))) {
                double overlap= iou(obj.obb, gtObj.obb);
                if (overlap){
                    foundInGlobalMap = 1;
                    state.isInGlobalMap = true;
                    state.isInWaitingList = false;
                    state.notInMap = false;
                    state.existenceProbability = obj.exist_certainty;
                    std::cout<< gtObj.name <<" is in global map."<< endl;
                }
            }
        }

        if(!foundInGlobalMap)
            state.isInGlobalMap = false;

        for (const auto& val : waitingObjectList) {
            const SemanticObject& obj = val.second;
            if (obj.exist_certainty > 0.25 && (obj.name == gtObj.name || similarClasses(obj.name, gtObj.name))) {
                double overlap= iou(obj.obb, gtObj.obb);
                if (overlap){
                    foundInWaitingList = 1;
                    state.isInGlobalMap = false;
                    state.isInWaitingList = true;
                    state.notInMap = false;
                    state.existenceProbability = obj.exist_certainty;
                    std::cout<< gtObj.name <<" is in waiting list."<< endl;
                }
            }
        }

        if(!foundInWaitingList)
            state.isInWaitingList = false;

        if(!state.isInGlobalMap && !state.isInWaitingList){
            state.notInMap = true;
            state.isInGlobalMap = false;
            state.isInWaitingList = false;
            if (previousNotInMap)
                state.existenceProbability = 0;
            else
                state.existenceProbability -= 0.25;
            std::cout << gtObj.name << " not visualized in map."<< endl;
        }

        if(previousNotInMap && (state.isInGlobalMap || state.isInWaitingList)){
            state.wasAdded = true;
            state.wasUpdated = false;
            state.wasRemoved = false;
        }

        if((previousIsInGlobalMap || previousIsInWaitingList) && state.wasMapped){
            state.wasAdded = false;
            state.wasUpdated = true;
            state.wasRemoved = false;
        }
        
        if((previousIsInGlobalMap || previousIsInWaitingList) && state.notInMap){
            state.wasAdded = false;
            state.wasUpdated = false;
            state.wasRemoved = true;
        }

        std::ofstream newfile;
        newfile.open(trackBackupFileName, std::ios::out | std::ios::app);
        newfile <<objectName<<","<<objectId<<","<<state.wasMapped<<","<<state.wasAdded<<","<<state.wasUpdated<<","<<state.wasRemoved<<","<<state.isInGlobalMap<<","<<state.isInWaitingList<<","<<state.notInMap<<","<<state.existenceProbability<<"\n";

        newfile.close();
    }

    int SemanticFusion::numberFalseNegativeInMap(std::map<size_t, SemanticObject> objectList, std::map<size_t, SemanticObject> groundTruthObjectList)
    {
        int n_false_negative = 0;
        for(auto &val : groundTruthObjectList){
            bool object_detected= false;
            SemanticObject &gtObj = val.second;
            for (auto &val2 : objectList){
                SemanticObject &obj = val2.second;
                double overlap= iou(obj.obb, gtObj.obb);
                if(obj.exist_certainty > 0.25 && obj.name == gtObj.name && overlap >= 0.5)
                    object_detected= true;
            }
            if(!object_detected)
                n_false_negative++;
        }
        return n_false_negative;
    }

    std::pair<int,int> SemanticFusion::numberTrueFalseDetectionInMap(std::map<size_t, SemanticObject> objectList, std::map<size_t, SemanticObject> groundTruthObjectList)
    {
        int n_true_positive = 0;
        int n_false_positive = 0;
        for(auto &val : objectList){
            SemanticObject &obj = val.second;
            if(inConsideredObjectList(obj.name) && obj.exist_certainty > 0.25){
                bool exist_in_ground_truth = false;
                for (auto &val2 : groundTruthObjectList){
                    SemanticObject &gtObj = val2.second;
                    double overlap= iou(obj.obb, gtObj.obb);
                    if((obj.name == gtObj.name) && overlap >= 0.5)
                        exist_in_ground_truth = true;
                }
                if(!exist_in_ground_truth)
                    n_false_positive++;
                else
                    n_true_positive++;
            }
        }
        std::pair<int,int> positive_detection;
        positive_detection.first= n_true_positive;
        positive_detection.second= n_false_positive;
        return positive_detection;
    }

    std::array<double,3> SemanticFusion::computeMapF1Score(double tp, double fp, double fn){
        double precision= tp/(tp+fp);
        double recall= tp/(tp+fn);
        double f1_score= 2* (precision*recall)/(precision+recall);
        std::array<double,3> array;
        array[0]= precision;
        array[1]= recall;
        array[2]= f1_score;
        return array;
    }
}