#ifndef SEM_FUSION_H
#define SEM_FUSION_H

#include <semanticmap.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>


using namespace Eigen;

namespace semmapping
{
    struct CategoryPriorKnowledge
    {
    std::string name;
    double length;
    double width;
    int objectNumber;
    };

    struct ObjectState
    {
    
    bool wasMapped = false;
    bool wasAdded = false;
    bool wasUpdated = false;
    bool wasRemoved = false;

    bool isInGlobalMap = false;
    bool isInWaitingList = false;
    bool notInMap = false;

    double existenceProbability = 0;
    };

    struct ClassStats {
    double true_positive = 0;
    double mean_iou = 0;
    double mean_com_offset = 0;
    double false_positive = 0;
    double false_negative = 0;
    double mean_orientation_offset = 0;
    double F1_score = 0;
    };

    class SemanticFusion
    {
        inline static double ref_fit(const polygon &newpg, const polygon &refpg)
        {
            multi_polygon sect;
            bg::intersection(newpg, refpg, sect);
            return bg::area(sect) / bg::area(refpg);
        }

        inline static double iou(const polygon &a, const polygon &b)
        {
            multi_polygon un;
            bg::union_(a, b, un);
            multi_polygon sect;
            bg::intersection(a, b, sect);
            //std::cout << "intersect: " << bg::area(sect) << " unioin: " <<bg::area(un) << std::endl;
            //intersection over union
            if(bg::area(un)!=0)
                return bg::area(sect) / bg::area(un);
            else
                return 0;
        }
        
        std::list<CategoryPriorKnowledge> CategoriesPriorKnowledge={};

        std::list<std::string> consideredCategories{"Chair","Table","Shelf", "Couch"}; //"Sofa bed"

        std::list<std::pair<std::string, int>> ObjectNumberInCategory{};

        std::pair<double, double> get_real_object_length_width(const std::string &name)
        {
            std::pair<double, double> dimensions;
            for (auto &val : CategoriesPriorKnowledge){
                if(val.name == name){
                    dimensions.first= val.length; 
                    dimensions.second= val.width;
                }
            }
            return dimensions;
        } 

        public:
            SemanticFusion();
            void show_map_id();
            bool setPriorKnowledge(std::istream &input);
            bool updatePriorKnowledge(std::istream &input);
            std::list<CategoryPriorKnowledge> getPriorKnowledge();
            CategoryPriorKnowledge getCategoryPriorKnowledge(std::string category);
            bool initializeCategoryObjectNumber();
            int getCategoryObjectNumber(std::string category);
            bool updateCategoryObjectNumber(std::string category, std::string incrementDecrement);
            void countObjectNumberPerCategoryInMap(semmapping::SemanticMap map);
            SemanticObject copySemanticObject(SemanticObject obj);
            SemanticObject nmsFusionOfSemanticObject(SemanticObject initial_obj, SemanticObject recieved_obj);
            double calculateOBBscore(const SemanticObject& obj1, const SemanticObject& obj2);
            SemanticObject objectWithObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
            SemanticObject objectWithObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj);
            SemanticObject objectWithoutObbUpdate(const SemanticObject& initial_obj, const SemanticObject& received_obj, semmapping::SemanticMap map);
            SemanticObject objectUpdate(SemanticObject initial_obj, SemanticObject received_obj, semmapping::SemanticMap global_map);
            SemanticObject objectUpdate(SemanticObject initial_obj, SemanticObject received_obj, semmapping::SemanticMap global_map, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
            bool similarClasses(std::string object1_name, std::string object_name2);
            bool checkTheAbilityOfObjectsToOverlap(std::string object1_name, std::string object_name2);
            bool inConsideredObjectList(std::string object_name);
            bool validObjectSurface(std::string object_name, polygon object_obb);
            bool isRectangular(polygon object_obb);
            point getObbLeftTop(polygon obb);
            point getObbRightTop(polygon obb);
            point getNearestPoint(point p, polygon poly);
            int getNearestPointIndex(point p, polygon poly);
            polygon reorderObbVertices(polygon to_reorder, polygon to_reorder_like);
            polygon computeMeanBox(polygon obb1, polygon obb2);
            polygon ekfRectangularBoxesFusion(polygon initial_obb, polygon new_obb, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
            void removeMapInconsistencies(semmapping::SemanticMap map, semmapping::SemanticMap &map_cleared, double overlap_threshold = 0.0);
            void semfusion(semmapping::SemanticMap previous_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map,
                double overlap_threshold = 0.0, std::string algorithm="our_solution");
            void semfusion_updated(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold = 0.0);
            void globalMapUpdate(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, semmapping::SemanticMap &waiting_objects, MatrixXd& P, MatrixXd& Q, MatrixXd& R, double overlap_threshold = 0.0);
            void semfusion_nms(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold = 0.5);
            void semfusion_modified_nms(semmapping::SemanticMap reference_map, semmapping::SemanticMap received_map, semmapping::SemanticMap &global_map, double overlap_threshold = 0.5);
            // Methods for evaluating merged maps
            bool isCloseDistance(double distance1, double distance2);
            double objectOrientation(polygon poly);
            void updateClassStats(ClassStats& stats, double overlap, double comOffset, double orientationOffset);
            void saveObjectStats(int id, std::string category, double overlap, double ComError, double orientationError, const std::string& filename);
            void saveMapStats(const std::map<std::string, ClassStats>& all_classes_stats, const std::string& filename);
            void evaluateSemanticMap(const std::map<size_t, SemanticObject>& objectList, const std::map<size_t, SemanticObject>& groundTruthObjectList, const std::string& backup_file_name);
            void trackGtObjectState(std::string objectName, int objectId, std::map<size_t, SemanticObject> groundTruthObjectList, std::map<size_t, SemanticObject> recievedObjectList, 
                std::map<size_t, SemanticObject> ObjectList, std::map<size_t, SemanticObject> waitingObjectList, ObjectState& state, std::string trackBackupFileName);
            void evaluateObjectCategoriesOverAllMaps(int mapID, std::map<std::string, ClassStats>& all_classes_stats, int& numberObjectsWithOBB, const std::map<size_t, SemanticObject>& objectList,
                const std::map<size_t, SemanticObject>& groundTruthObjectList, const std::string& backup_file_name);

            // Methods to compute F1 score of a map
            int numberFalseNegativeInMap(std::map<size_t, SemanticObject> objectList, std::map<size_t, SemanticObject> groundTruthObjectList);
            std::pair<int,int> numberTrueFalseDetectionInMap(std::map<size_t, SemanticObject> objectList, std::map<size_t, SemanticObject> groundTruthObjectList);
            std::array<double,3> computeMapF1Score(double tp, double fp, double fn); //function return precision, recall and F1 score of the map
    };
}

#endif // SEM_FUSION_H