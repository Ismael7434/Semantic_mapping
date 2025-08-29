#include "semanticmap.h"
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <array>


namespace semmapping
{

    SemanticMap::SemanticMap(tf2_ros::Buffer &tfBuffer, pcl::visualization::PCLVisualizer* &viewer, boost::mutex &viewer_mtx, int &semmap_vport0, int &semmap_vport1, semmapping::ParamsConfig &param_config): tfBuffer(tfBuffer), viewer(viewer), viewer_mtx(viewer_mtx), semmap_vport0(semmap_vport0), semmap_vport1(semmap_vport1), param_config(param_config)
    {
    }
    


    std::map<std::string, std::tuple<uint8_t, uint8_t, uint8_t>> classColorMap = {
        { "Table", std::make_tuple(0, 255, 26) },   // vert fluo
        { "Chair", std::make_tuple(255, 255, 0) },   // red
        { "Bed", std::make_tuple(0, 0, 139) },   // Blue foncé
        { "Couch", std::make_tuple(255, 192, 203) },  // Rose
        { "Trash can", std::make_tuple(255, 0, 0) }  // Jaune vif
    };

    bool SemanticMap::similarClasses(std::string object1_name, std::string object2_name)
    {
        if ((object1_name == "Sofa bed" && object2_name == "Couch") || (object1_name == "Couch" && object2_name == "Sofa bed"))
            return true;
        else
            return false;
    }

    bool SemanticMap::checkTheAbilityOfObjectsToOverlap(std::string object1_name, std::string object2_name){
        /*if(object1_name == object2_name)
            return true;*/
        if(similarClasses(object1_name, object2_name))
            return true;
        else if ((object1_name == "Chair" && object2_name == "Table") || (object1_name == "Table" && object2_name == "Chair"))
            return true;
        else
            return false;
    }
    
    void SemanticMap::filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg)
    {
        ROS_INFO("Filtering objects");
        for (auto it = object_list.begin(); it != object_list.end(); )
        {
            const SemanticObject &obj = objectList.at(*it);
            if (union_fit(pg, obj.shape_union) < FIT_THRESH)// || bg::covered_by(obj.shape_union,pg))
            {
                it = object_list.erase(it);
                ROS_INFO("Object removed");
                continue;
            }
            it++;
        }
    }
        std::vector<std::pair<double, double>> SemanticMap::get_object_contour(const std::string& name)
    {
        std::vector<std::pair<double, double>> contour;
        std::string path = "/home/ismael/basedeconnaissance/";
        std::string filename;

        if (name == "Chair") filename = "chaise.txt";
        else if (name == "Table") filename = "table.txt";
        else if (name == "Shelf") filename = "shelf.txt";
        else if (name == "Couch") filename = "couch.txt";
        else if (name == "Bed") filename = "bed.txt";
        else if (name == "Trash can") filename = "Trash can.txt";
        else {
            ROS_WARN("Aucun contour connu pour l'objet : %s", name.c_str());
            return contour;  // contour vide
        }

        std::ifstream infile(path + filename);
        if (!infile.is_open()) {
            ROS_WARN("Impossible d'ouvrir le fichier de contour : %s", (path + filename).c_str());
            return contour;
        }

        double x, y;
        while (infile >> x >> y) {
            contour.emplace_back(x, y);
        }

        size_t nb_points = contour.size();
        size_t nb_segments = nb_points;

        // --- Nouveau calcul du centroïde avec Boost.Geometry ---
        namespace bg = boost::geometry;
        typedef bg::model::d2::point_xy<double> BoostPoint;
        typedef bg::model::polygon<BoostPoint> BoostPolygon;

        BoostPolygon poly;
        for (const auto& pt : contour) {
            bg::append(poly.outer(), BoostPoint(pt.first, pt.second));
        }

        if (!contour.empty() && !bg::equals(poly.outer().front(), poly.outer().back())) {
            poly.outer().push_back(poly.outer().front());
        }

        BoostPoint centroid;
        bg::centroid(poly, centroid);
        double centroid_x = centroid.x();
        double centroid_y = centroid.y();
        // -------------------------------------------------------

        ROS_INFO(" Contour chargé pour l'objet '%s' | Sommets : %lu | Segments : %lu | Centroïde : (%.2f, %.2f)",
                name.c_str(), nb_points, nb_segments, centroid_x, centroid_y);

        return contour;
    }

    double SemanticMap::distanceFromLine(point p, point start, point end) {
        // function to calculate the distance of a point from a line
        double numerator = abs((end.y() - start.y())*p.x() - (end.x() - start.x())*p.y() + end.x()*start.y() - end.y()*start.x());
        double denominator = sqrt(pow(end.y() - start.y(), 2) + pow(end.x() - start.x(), 2));
        return numerator / denominator;
    }

    polygon SemanticMap::improve_polygon(polygon poly, double tolerance){
        ROS_INFO(" Appel de improve_polygon() pour le filtrage des sommets");
        polygon improved_polygon;
        int j=0;
        
            // Sauvegarde du polygone original
        std::ofstream poly_in("/home/ismael/Documents/detected_polygons/poly_Avant_filtre.txt");
        if (poly_in.is_open()) {
            ROS_INFO(" Enregistrement du polygone Avant filtrage (%lu sommets)", poly.outer().size());
            for (const auto& pt : poly.outer()) {
                poly_in << pt.x() << " " << pt.y() << "\n";
            }
            poly_in.close();

            // Affichage des sommets dans le terminal
            ROS_INFO("Sommets du polygone avant filtrage :");
            for (const auto& pt : poly.outer()) {
                ROS_INFO(" - (%.3f, %.3f)", pt.x(), pt.y());
            }

                    // Calcul et affichage du centroïde
            point centroid;
            bg::centroid(poly, centroid);
            ROS_INFO(" Centroide du polygone : (%.3f, %.3f)", centroid.x(), centroid.y());

        } else {
            ROS_WARN("Impossible d'ouvrir poly__Avant_filtre.txt pour écrire.");
        }
        bg::append(improved_polygon.outer(), poly.outer()[0]);
        for(int i=0; i< poly.outer().size()-2; i++){
            double d= distanceFromLine(poly.outer()[i+j+1], poly.outer()[i], poly.outer()[i+j+2]);
            if(d < tolerance){
                i--;
                j++;
            } 
            else
            {
                if(i+j+1<poly.outer().size()-1){
                    bg::append(improved_polygon.outer(), poly.outer()[i+j+1]);
                    i=i+j;
                    j=0;
                }
                else
                    i=i+j;
            }
        }
        bg::append(improved_polygon.outer(), poly.outer()[poly.outer().size()-1]);
            //  Sauvegarde du polygone amélioré
        std::ofstream poly_out("/home/ismael/Documents/detected_polygons/poly_filtre.txt");
        if (poly_out.is_open()) {
            ROS_INFO("Enregistrement du polygone Filtré (%lu sommets)", improved_polygon.outer().size());
            for (const auto& pt : improved_polygon.outer()) {
                poly_out << pt.x() << " " << pt.y() << "\n";
            }
            poly_out.close();

            //  Affichage des sommets dans le terminal
            ROS_INFO(" Sommets du polygone Filtré :");
            for (const auto& pt : improved_polygon.outer()) {
                ROS_INFO(" - (%.3f, %.3f)", pt.x(), pt.y());
            }
        } else {
            ROS_WARN(" Impossible d'ouvrir poly_filtre.txt pour ecrire.");
        }

        return improved_polygon;
    }

    std::list<std::pair<point, point>> SemanticMap::get_polygon_first_plan_edges(polygon poly, point reference){
        ROS_INFO("Appel de get_polygon_first_plan_edges() pour la detection des cotes premiers plans");
        std::list<std::pair<point, point>> first_plan_edges_list;

        for(int i=0; i< poly.outer().size()-1; i++){
            double edge_distance = sqrt((poly.outer()[i+1].x() - poly.outer()[i].x())*(poly.outer()[i+1].x() - poly.outer()[i].x()) 
                                + (poly.outer()[i+1].y() - poly.outer()[i].y())*(poly.outer()[i+1].y() - poly.outer()[i].y()));
            //cout<<"("<<poly.outer()[i].x()<<","<<poly.outer()[i].y()<<") et ("<<poly.outer()[i+1].x()<<","<<poly.outer()[i+1].y()<<") edge_distance= "<<edge_distance<<endl;
            if(edge_distance > 0.1){
                polygon from_reference_to_edge_area;
                bg::append(from_reference_to_edge_area.outer(), reference);
                bg::append(from_reference_to_edge_area.outer(), poly.outer()[i]);
                bg::append(from_reference_to_edge_area.outer(), poly.outer()[i+1]);
                bg::append(from_reference_to_edge_area.outer(), reference);
                
                if (!bg::is_valid(from_reference_to_edge_area)){
                    from_reference_to_edge_area.clear();
                    bg::append(from_reference_to_edge_area.outer(), reference);
                    bg::append(from_reference_to_edge_area.outer(), poly.outer()[i+1]);
                    bg::append(from_reference_to_edge_area.outer(), poly.outer()[i]);
                    bg::append(from_reference_to_edge_area.outer(), reference);
                }

                multi_polygon intersection;
                bg::intersection(from_reference_to_edge_area, poly, intersection);
                if(bg::area(intersection)== 0){
                    std::pair<point, point> edge;
                    edge.first= poly.outer()[i];
                    edge.second= poly.outer()[i+1];
                    //cout<<"edge= ("<<edge.first.x()<<","<<edge.first.y()<<") - ("<<edge.second.x()<<","<<edge.second.y()<<") edge distance= "<<edge_distance<<endl;
                    first_plan_edges_list.push_back(edge);
                } 
            }
        }
            // Enregistrement des segments visibles
        std::ofstream segfile("/home/ismael/Documents/detected_polygons/segments_premier_plan.txt");
        if (segfile.is_open()) {
            ROS_INFO(" Segments de premier plan détectés (%lu) :", first_plan_edges_list.size());
            for (const auto& edge : first_plan_edges_list) {
                segfile << edge.first.x() << " " << edge.first.y() << " "
                        << edge.second.x() << " " << edge.second.y() << "\n";
                ROS_INFO(" - (%.3f, %.3f) → (%.3f, %.3f)",
                        edge.first.x(), edge.first.y(),
                        edge.second.x(), edge.second.y());
            }
            segfile.close();
        } else {
            ROS_WARN(" Impossible d'enregistrer les segments dans segments_premier_plan.txt");
        }
        return first_plan_edges_list;
    }

    std::list<std::pair<point, point>> SemanticMap::get_association_valid_edges(polygon poly){
        std::list<std::pair<point, point>> association_edges_list;
        for(int i=0; i< poly.outer().size()-1; i++){
            double edge_distance = sqrt((poly.outer()[i+1].x() - poly.outer()[i].x())*(poly.outer()[i+1].x() - poly.outer()[i].x()) 
                                + (poly.outer()[i+1].y() - poly.outer()[i].y())*(poly.outer()[i+1].y() - poly.outer()[i].y()));
            if(edge_distance > 0.01){
                std::pair<point, point> edge;
                edge.first= poly.outer()[i];
                edge.second= poly.outer()[i+1];
                association_edges_list.push_back(edge);
            }
        }
        return association_edges_list;
    }
    void SemanticMap::associate_real_box_to_partial_polygon(
        polygon poly,
        std::list<std::pair<point, point>> first_plan_edges,
        const std::string& name,
        std::vector<std::pair<polygon, double>>& selected_obb_list)
    {
        // ➤ Début chrono
        auto t_start = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM(" Appel de la methode d'association pour l'objet : " << name);
        std::vector<std::pair<double, double>> model_points = get_object_contour(name);
        if (model_points.size() < 2) return;
    
        // Convertir les points du modèle en polygon Boost
        polygon model_poly;
        for (auto& p : model_points)
            bg::append(model_poly.outer(), point(p.first, p.second));
        if (model_points.front() != model_points.back())
            bg::append(model_poly.outer(), point(model_points.front().first, model_points.front().second));
        bg::correct(model_poly); // corriger orientation

                // Affichage du polygone modèle (model_poly)
        ROS_INFO_STREAM("Polygone modèle : " << name << ", " << model_poly.outer().size() << " sommets");
        for (const auto& pt : model_poly.outer()) {
            ROS_INFO_STREAM(" - (" << pt.x() << ", " << pt.y() << ")");
        }

        // Sauvegarde du polygone modèle
        std::string model_output_path = "/home/ismael/Documents/detected_polygons/model_polygon_" + name + ".txt";
        std::ofstream model_file(model_output_path);
        if (model_file.is_open()) {
            for (const auto& pt : model_poly.outer()) {
                model_file << pt.x() << " " << pt.y() << std::endl;
            }
            model_file.close();
            ROS_INFO_STREAM("Polygone modèle sauvegardé dans : " << model_output_path);
        } else {
            ROS_ERROR_STREAM("Impossible d’écrire dans : " << model_output_path);
        }
    
        double best_score = -1;
        std::pair<polygon, double> best_hypothesis;
    
        for (std::size_t i = 0; i < model_poly.outer().size(); ++i)
        {
            point A1 = model_poly.outer()[i];
            point A2 = model_poly.outer()[(i + 1) % model_poly.outer().size()];
            ROS_INFO_STREAM("Modèle - Edge A1: (" << A1.x() << ", " << A1.y() << ") → A2: (" << A2.x() << ", " << A2.y() << ")");
            /*for (std::size_t j = 0; j < poly.outer().size(); ++j)
            {
                point S1 = poly.outer()[j];
                point S2 = poly.outer()[(j + 1) % poly.outer().size()];*/
            for (const auto& obs_edge_raw : first_plan_edges)
            {
                point S1 = obs_edge_raw.first;
                point S2 = obs_edge_raw.second;
    
                // Étape 0 — calcul des longueurs des vecteurs
                double dx1 = A2.x() - A1.x();
                double dy1 = A2.y() - A1.y();
                double dx2 = S2.x() - S1.x();
                double dy2 = S2.y() - S1.y();
    
                // Assurer que les deux vecteurs vont dans la même direction
                double dot = dx1 * dx2 + dy1 * dy2;
                if (dot < 0.0) {
                    std::swap(S1, S2);
                    dx2 = S2.x() - S1.x();
                    dy2 = S2.y() - S1.y();
                    ROS_INFO_STREAM(" Observation - Edge S1 = (" << S1.x() << ", " << S1.y() << "), S2 = (" << S2.x() << ", " << S2.y() << ")");
                } else {
                        ROS_INFO_STREAM("Observation - Edge : S1 = (" << S1.x() << ", " << S1.y() << "), S2 = (" << S2.x() << ", " << S2.y() << ")");
                }
    
                double len_model = std::hypot(dx1, dy1);
                double len_obs = std::hypot(dx2, dy2);
                if (len_model < 1e-6) continue;
    
                double angle = atan2(dy2, dx2) - atan2(dy1, dx1);
                double cos_theta = cos(angle);
                double sin_theta = sin(angle);
                ROS_INFO_STREAM("Angle entre vecteurs (rad) : " << angle << " — cos: " << cos_theta << ", sin: " << sin_theta);
    
                // Transformation (rotation + translation)
                polygon transformed_model;
                for (const auto& pt : model_poly.outer()) {
                    double x_rot = A1.x() + cos_theta * (pt.x() - A1.x()) - sin_theta * (pt.y() - A1.y());
                    double y_rot = A1.y() + sin_theta * (pt.x() - A1.x()) + cos_theta * (pt.y() - A1.y());
    
                    double x_tr = x_rot + (S1.x() - A1.x());
                    double y_tr = y_rot + (S1.y() - A1.y());
    
                    bg::append(transformed_model.outer(), point(x_tr, y_tr));
                                    // Log pour vérifier si le premier point du modèle tourné (A1) est bien aligné avec S1
                    if (pt.x() == A1.x() && pt.y() == A1.y()) {
                        ROS_INFO_STREAM(" A1 (" << pt.x() << ", " << pt.y() << ") transformé en S1 : (" << x_tr << ", " << y_tr << ")");
                        ROS_INFO_STREAM("Observation - Edge S1: (" << S1.x() << ", " << S1.y() << ") → S2: (" << S2.x() << ", " << S2.y() << ")");
                    }
                }
    
                if (!bg::equals(transformed_model.outer().front(), transformed_model.outer().back()))
                    bg::append(transformed_model.outer(), transformed_model.outer().front());
                bg::correct(transformed_model);
                                /* Affichage et sauvegarde du polygone transformé
                ROS_INFO_STREAM("Polygone_model transformé (aligné) : " << name << ", " << transformed_model.outer().size() << " sommets");
                for (const auto& pt : transformed_model.outer()) {
                    ROS_INFO_STREAM(" - (" << pt.x() << ", " << pt.y() << ")");
                }*/

                                // Compteur d’hypothèses
                static int hypo_count = 0;

                // Générer un nom de fichier unique pour chaque hypothèse
                std::stringstream ss;
                ss << "/home/ismael/Documents/detected_polygons/hypothese/transformed_model_polygon_" << name << "_hypo_" << hypo_count << ".txt";
                std::string transformed_output_path = ss.str();

                // Sauvegarde du polygone transformé
                std::ofstream transformed_file(transformed_output_path);
                if (transformed_file.is_open()) {
                    for (const auto& pt : transformed_model.outer()) {
                        transformed_file << pt.x() << " " << pt.y() << std::endl;
                    }
                    transformed_file.close();
                    ROS_INFO_STREAM("Hypothèse #" << hypo_count << " sauvegardée dans : " << transformed_output_path);
                } else {
                    ROS_ERROR_STREAM("Impossible d’écrire dans : " << transformed_output_path);
                }

                // Incrémenter le compteur pour la prochaine hypothèse
                hypo_count++;
    
                // Évaluation du score
                double score_length = 1.0 - std::abs(len_model - len_obs) / std::max(len_model, len_obs);
    
                point cen_model, cen_obs;
                bg::centroid(transformed_model, cen_model);
                bg::centroid(poly, cen_obs);
                double center_dist = std::hypot(cen_model.x() - cen_obs.x(), cen_model.y() - cen_obs.y());
                double score_center = 1.0 / (1.0 + center_dist);
    
                int aligned_count = 0;
                auto model_edges = get_association_valid_edges(transformed_model);
                double angle_threshold = 0.05;
                double distance_threshold = 0.05;
    
                for (const auto& edge_obs : first_plan_edges)
                {
                    point P1 = edge_obs.first;
                    point P2 = edge_obs.second;
                    double angle_obs = atan2(P2.y() - P1.y(), P2.x() - P1.x());
    
                    for (const auto& edge_model : model_edges)
                    {
                        point Q1 = edge_model.first;
                        point Q2 = edge_model.second;
                        double angle_model = atan2(Q2.y() - Q1.y(), Q2.x() - Q1.x());
    
                        double angle_diff = std::abs(angle_obs - angle_model);
                        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
    
                        bg::model::linestring<point> seg_obs{P1, P2};
                        bg::model::linestring<point> seg_model{Q1, Q2};
                        double dist = bg::distance(seg_obs, seg_model);
    
                        if (angle_diff < angle_threshold && dist < distance_threshold)
                        {
                            aligned_count++;
                            break;
                        }
                    }
                }
    
                double score_align = static_cast<double>(aligned_count) / std::max((int)first_plan_edges.size(), 1);
                double total_score = 0.2 * score_length + 0.2 * score_center + 0.6 * score_align;
                ROS_INFO_STREAM("Association avec modèle " << name);
                ROS_INFO_STREAM("score_length  (poids 0.2) = " << score_length);
                ROS_INFO_STREAM("score_center  (poids 0.2) = " << score_center);
                ROS_INFO_STREAM("score_align   (poids 0.6) = " << score_align);
                ROS_INFO_STREAM("score_total            = " << total_score);
    
                if (total_score > best_score)
                {
                    best_score = total_score;
                    best_hypothesis.first = transformed_model;
                    best_hypothesis.second = total_score;
                }
            }
        }
    
        if (best_score >=0.5)
        {
            selected_obb_list.push_back(best_hypothesis);
            ROS_INFO_STREAM("Association retenue pour " << name << " avec score : " << best_score);

                    // Formater le score avec 2 décimales pour le nom de fichier
            std::ostringstream score_stream;
            score_stream << std::fixed << std::setprecision(2) << best_score;
            std::string score_str = score_stream.str();

            // Remplacer le point par un underscore dans le nom du fichier (optionnel, pour éviter les soucis système)
            std::replace(score_str.begin(), score_str.end(), '.', '_');

            // Construire le chemin avec le score
            std::string output_path = "/home/ismael/Documents/detected_polygons/score/best_alignment_" + name + "_score" + score_str + ".txt";

            std::ofstream outfile(output_path);
            if (outfile.is_open()) {
                for (const auto& pt : best_hypothesis.first.outer()) {
                    outfile << pt.x() << " " << pt.y() << std::endl;
                }
                outfile.close();
                ROS_INFO_STREAM("Polygone aligné sauvegardé dans : " << output_path);
            } else {
                ROS_ERROR_STREAM("Impossible d’écrire dans : " << output_path);
            }
        }
        else
        {
            ROS_WARN_STREAM("Aucune association satisfaisante pour " << name << " (score max : " << best_score << ")");
        }
            // ➤ Fin chrono et enregistrement
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        std::ofstream timefile("/home/ismael/Documents/detected_polygons/time/timing_association.txt", std::ios::app);
        if (timefile.is_open()) {
            timefile << name << "," << elapsed_ms << " ms\n";
            timefile.close();
        } else {
            ROS_ERROR_STREAM("Impossible d’écrire dans le fichier de temps.");
        }

        ROS_INFO_STREAM("Temps de calcul pour " << name << " : " << elapsed_ms << " ms");
    }
    std::pair<polygon, double> SemanticMap::create_object_box_using_prior_knowledge(polygon poly, const std::string &name, bool use_first_plan_edges)
    {
        ROS_INFO_STREAM("=== [OBJECT: " << name << "] ===");

        // Étape 1 : amélioration du polygone d'entrée
        number_of_initial_edges += poly.outer().size() - 1;
        polygon poly_brut = poly;
        poly = improve_polygon(poly, 0.02);
        number_of_edges_after_filtering += poly.outer().size() - 1;
        ROS_INFO_STREAM("Detected shape area = " << bg::area(poly));

        // Étape 2 : récupérer les arêtes pertinentes (selon l’option use_first_plan_edges)
        std::list<std::pair<point, point>> association_edges_list;
        if (use_first_plan_edges) {
            point robot_position = getRobotPosition();
            association_edges_list = get_polygon_first_plan_edges(poly, robot_position);
        } else {
            association_edges_list = get_association_valid_edges(poly);
        }
        number_of_processed_edges += association_edges_list.size();
                // === Journalisation des statistiques dans le fichier TXT ===
        int n_avant_filtrage = number_of_initial_edges;
        int n_apres_filtrage = number_of_edges_after_filtering;
        int n_premier_plan = association_edges_list.size();

        std::ofstream stats_log("/home/ismael/Documents/detected_polygons/statistiques_segments.txt", std::ios::app);
        if (stats_log.is_open()) {
            stats_log << name << ","
                    << n_avant_filtrage << ","
                    << n_apres_filtrage << ","
                    << n_premier_plan << "\n";
            stats_log.close();
            ROS_INFO_STREAM(" Statistiques enregistrées pour l’objet " << name);
        } else {
            ROS_WARN_STREAM("Impossible d’ouvrir le fichier de statistiques.");
        }

        // Étape 3 : appel à la nouvelle méthode d’association avec contour réel
        std::vector<std::pair<polygon, double>> selected_obb_list;
        associate_real_box_to_partial_polygon(poly, association_edges_list, name, selected_obb_list);

        // Étape 4 : sélection de la meilleure hypothèse selon le score
        if (!selected_obb_list.empty()) {
            std::pair<polygon, double> best_obb = selected_obb_list[0];
            for (const auto& candidate : selected_obb_list) {
                if (candidate.second > best_obb.second)  // plus grand score = meilleure association
                    best_obb = candidate;
            }

            bg::correct(best_obb.first);
            ROS_INFO_STREAM("→ Meilleure hypothèse retenue avec score = " << best_obb.second);
            return best_obb;
        }

        //Aucun appariement satisfaisant → on renvoie le polygone observé
        ROS_WARN_STREAM("→ Aucun appariement satisfaisant pour " << name << ". Utilisation du polygone détecté brut.");
        std::pair<polygon, double> fallback;
        fallback.first = poly_brut;
        fallback.second = 0.0;
        return fallback;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromInd(pcl::PointIndices::Ptr input_indices,  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(input_cloud);
        extract.setIndices(input_indices);
        extract.filter(*result_cloud);
        return result_cloud;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> SemanticMap::getObjectPointsReg(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> return_cloud;
        std::vector<pcl::PointIndices> cluster_indices;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster;
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);


        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (cloud);

        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (20.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (cluster_indices);

        if (cluster_indices.size() == 0)
        {
            ROS_WARN("No points in cluster indicies");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
        }
        for(int i=0;i<cluster_indices.size();i++){
            pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
            temp_indices->indices = cluster_indices[i].indices;
            point_cloud_cluster.push_back(*pointCloudFromInd(temp_indices,cloud));
        }
        return point_cloud_cluster;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> SemanticMap::getObjectPointsEuc(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster;
        pcl::PointCloud<pcl::PointXYZ> return_cloud;
        std::vector<pcl::PointIndices> cluster_indices;


        if (cloud->points.size() == 0)
        {
            ROS_WARN("No points in cloud before Euclead and tree");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
            return point_cloud_cluster;
        }
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        //Do eucledean clustering
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1);
        ec.setMinClusterSize (double(100));
        ec.setMaxClusterSize (cloud->points.size());
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        if (cluster_indices.size() == 0)
        {
            ROS_WARN("No points in cluster indicies");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
        }
        for(int i=0;i<cluster_indices.size();i++){
            pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
            temp_indices->indices = cluster_indices[i].indices;
            point_cloud_cluster.push_back(*pointCloudFromInd(temp_indices,cloud));
        }
        return point_cloud_cluster;
    }

    double calculateMeanHight(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input_cloud,centroid);
        return centroid[2];
        }
    // Combine shapes
    void SemanticMap::updateUnion(size_t id)
    {
        SemanticObject &obj = objectList.at(id);
    
        if (obj.shapes.size() < 1)
        {
            ROS_ERROR("Semantic object has no shape");
            return;
        }
    
        // Étape 1 : générer la forme d’union des points
        obj.shape_union = computeConvexHullPcl(obj.point_cloud);
    
        // Étape 2 : si la forme semble incohérente (ex. trop grande), on filtre
        if (bg::area(obj.shape_union) > 50.0) // Seuil arbitraire, à adapter selon ton environnement
        { 
            obj.times_merged = 0;
    
            // Nettoyage des points
            obj.point_cloud = removeOutliers(obj.point_cloud);
    
            // Regroupement en clusters
            std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster = getObjectPointsEuc(obj.point_cloud);
    
            // Garder uniquement le plus gros cluster
            pcl::PointCloud<pcl::PointXYZ>::Ptr biggest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            size_t biggest_cluster_size = 0;
            for (const auto& cluster : point_cloud_cluster) {
                if (cluster.size() > biggest_cluster_size) {
                    *biggest_cluster = cluster;
                    biggest_cluster_size = cluster.size();
                }
            }
    
            // Mise à jour
            obj.point_cloud = biggest_cluster;
            obj.shape_union = computeConvexHullPcl(obj.point_cloud);
        }
    
        // Étape 3 : association géométrique avec la base de connaissances
        std::pair<polygon, double> selected_obb = create_object_box_using_prior_knowledge(obj.shape_union, obj.name);
    
        obj.obb = selected_obb.first;
        obj.obb_score = selected_obb.second;
    
        // Centroïdes
        bg::centroid(obj.obb, obj.oriented_box_cen);
        bg::centroid(obj.shape_union, obj.shape_union_cen);
    
        // Mise à jour dans l’arbre spatial
        objectRtree.remove(std::make_pair(obj.bounding_box, id));
        obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
        objectRtree.insert(std::make_pair(obj.bounding_box, id));
    }
    
    size_t SemanticMap::combineObjects(std::set<size_t> objects)
    {
        ROS_INFO_STREAM("Combining " << objects.size() << " objects");
        auto it = objects.begin();
        size_t combinedId = *it;
        it++;
        SemanticObject &combinedObj = objectList.at(combinedId);

        for (; it != objects.end(); it++)
        {
            SemanticObject &toMerge = objectList.at(*it);

            // Fusion des nuages de points
            *combinedObj.point_cloud += *toMerge.point_cloud;

            // Moyenne des attributs
            combinedObj.mean_height = (combinedObj.mean_height + toMerge.mean_height) / 2.0;
            combinedObj.class_confidence = (combinedObj.class_confidence + toMerge.class_confidence) / 2.0;

            // Conserver la plus grande file de comptage
            if (toMerge.counting_queue.size() > combinedObj.counting_queue.size())
                combinedObj.counting_queue = toMerge.counting_queue;

            // Suppression de l’objet fusionné
            removeObject(*it);
        }

        // Mise à jour complète de l’objet fusionné
        updateUnion(combinedId);

        return combinedId;
    }


    // Add new object or update existing object
    void SemanticMap::addEvidence(const std::string &name, const float &confidence, const polygon &pg, double mean_height, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        
        std::set<size_t> existingObjects = getObjectsByNameInRange(name, pg);

        //filter all neighbors with IoU < 0.2
        //filterIntersectionThresh(existingObjects, pg);
        if (existingObjects.empty()) 
        {
            // no object evidence exists yet, create new
            ROS_INFO("No Neighbors fitting, Create new object");
            addNewObject(name,confidence, pg, mean_height, cloud);
        } 
        else 
        {
            size_t objectId = *existingObjects.begin();

            // if more than one object fits, combine objects
            if (existingObjects.size() > 1) {
                objectId = combineObjects(existingObjects);
            }

            // combine combinedShape with new one
            SemanticObject &obj = objectList.at(objectId);

            // For existence prob update: if queue size is > 30 pop first entry and push 1
            if (obj.counting_queue.size() > param_config.queue_size) obj.counting_queue.pop();
            obj.counting_queue.push(1);
            obj.times_merged++;

            obj.shape_union = computeConvexHullPcl(obj.point_cloud);
            obj.isCombined = true;

            // Object occupancy zone update if the object shape shows important change
            polygon newCloudpolygon = computeConvexHullPcl(cloud);
            double overlap = iou(obj.shape_union, newCloudpolygon);

            if(overlap < 0.9){
                obj.class_confidence = (obj.class_confidence + confidence)/ 2;
                obj.mean_height = (obj.mean_height + mean_height) / 2;
                *obj.point_cloud += *cloud;

                pcl::VoxelGrid <pcl::PointXYZ> vox;
                vox.setInputCloud(obj.point_cloud);
                vox.setLeafSize(0.02, 0.02, 0.02);
                vox.setSaveLeafLayout(true);
                vox.filter(*obj.point_cloud);

                updateUnion(objectId);
            }
        }

        pcl::PassThrough<pcl::PointXYZ> pass;
        
        // check if the object polygon lies on table and filter table plan
        for (size_t table_id : tableList){
            SemanticObject &table = objectList.at(table_id);
            std::set<size_t> tableObjects = getObjectsInRange(table.shape_union);
            for (size_t id : tableObjects)
            {
                SemanticObject &tableObj = objectList.at(id);
                if (tableObj.name == "Table"){
                    continue;
                }
                pass.setInputCloud (tableObj.point_cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (table.mean_height-0.20, table.mean_height+0.02);
                pass.setNegative (true);
                pass.filter(*tableObj.point_cloud);

                if(tableObj.point_cloud->points.size() < 5)
                {
                    ROS_INFO_STREAM("Object " << tableObj.name << " removed beacause it was in table points");
                    removeObject(id);
                }
                else
                {
                    tableObj.shape_union = computeConvexHullPcl(tableObj.point_cloud);
                    std::pair<polygon, double> selected_obb;
                    selected_obb = create_object_box_using_prior_knowledge(tableObj.shape_union, tableObj.name);
                    tableObj.obb = selected_obb.first;
                    tableObj.obb_score = selected_obb.second;
                }
            }
        }
    }

    void SemanticMap::updateEvidenceCertainty(const std::string &name, const polygon &pg)
    {
        std::set<size_t> existingObjects = getObjectsByNameInRange(name, pg);
        if (!existingObjects.empty()) 
        {
            size_t objectId = *existingObjects.begin();

            if (existingObjects.size() > 1) {
                objectId = combineObjects(existingObjects);
            }

            SemanticObject &obj = objectList.at(objectId);

            //if queue size is > 30 pop first entry and push 1
            if (obj.counting_queue.size() > param_config.queue_size) obj.counting_queue.pop();
            obj.counting_queue.push(1);

            obj.isCombined = true;
        }
    }
    
    double calculateCertainty(std::queue<int> counting_queue){
        int hit = 0;
        int miss = 0;
        std::queue <int> temp = counting_queue;
        while (!temp.empty())
        {
            if (temp.front() == 1) hit+=1;
            else if(temp.front() == 0) miss+=1;
            temp.pop();
        }
        return (double)hit/(double)(hit+ miss + (1./counting_queue.size()));
    }
    
   void SemanticMap::removeEvidence(const polygon &visibilityArea, const point &robot)
    {
       std::set<size_t> existingObjects = getObjectsWithinRange(visibilityArea);
   
       for (size_t id : existingObjects)
       {
           SemanticObject &obj = objectList.at(id);
   
           point cen_point;
           bg::centroid(obj.shape_union, cen_point);
           double dist_to_obj = bg::distance(robot, cen_point);
           double obj_area = bg::area(obj.shape_union);
   
           multi_polygon ObjectPartInVisibilityArea;
           bg::intersection(visibilityArea, obj.shape_union, ObjectPartInVisibilityArea);
           double objectCoveredArea = bg::area(ObjectPartInVisibilityArea) / obj_area;
   
           //  Skip objects not well visible or out of range (if not combined)
           if ((dist_to_obj > 3.0 || dist_to_obj < 1.7 || objectCoveredArea < 0.8) && !obj.isCombined)
               continue;
   
           // ✅ Mise à jour pour les objets combinés
           if (obj.isCombined)
           {
               obj.exist_certainty = calculateCertainty(obj.counting_queue);
           }
           else
           {
               bool hasVisibleEdge = false;
   
               // ⚠ Vérification sécurité sur obj.obb
               if (!bg::is_valid(obj.obb)) {
                   ROS_WARN_STREAM("Polygon in obj.obb is invalid for object " << obj.name);
                   continue;
               }
   
               const auto &border = obj.obb.outer();
   
               for (size_t i = 0; i < border.size() - 1; ++i)
               {
                   polygon area_from_robot_to_edge;
                   bg::append(area_from_robot_to_edge.outer(), robot);
                   bg::append(area_from_robot_to_edge.outer(), border[i]);
                   bg::append(area_from_robot_to_edge.outer(), border[i + 1]);
                   bg::append(area_from_robot_to_edge.outer(), robot);
   
                   // 🛠 Correction si nécessaire
                   if (!bg::is_valid(area_from_robot_to_edge))
                   {
                       area_from_robot_to_edge.clear();
                       bg::append(area_from_robot_to_edge.outer(), robot);
                       bg::append(area_from_robot_to_edge.outer(), border[i + 1]);
                       bg::append(area_from_robot_to_edge.outer(), border[i]);
                       bg::append(area_from_robot_to_edge.outer(), robot);
                   }
   
                   multi_polygon intersection;
                   bg::intersection(area_from_robot_to_edge, obj.obb, intersection);
   
                   if (!bg::area(intersection))  // l'arête est visible
                   {
                       hasVisibleEdge = true;
   
                       // Vérifier si cette visibilité est bloquée par un autre objet
                       for (size_t id2 : existingObjects)
                       {
                           if (id2 == id) continue;
   
                           SemanticObject &obj2 = objectList.at(id2);
                           if (!checkTheAbilityOfObjectsToOverlap(obj.name, obj2.name))
                               break;
   
                           multi_polygon intersection2;
                           bg::intersection(area_from_robot_to_edge, obj2.obb, intersection2);
   
                           if (bg::area(intersection2))
                           {
                               hasVisibleEdge = false;
                               break;
                           }
                       }
   
                       if (hasVisibleEdge)
                           break;
                   }
               }
   
               // Réduction de la certitude si une arête visible
               if (hasVisibleEdge)
               {
                   if (obj.counting_queue.size() > param_config.queue_size)
                       obj.counting_queue.pop();
                   obj.counting_queue.push(0);
                   obj.exist_certainty = calculateCertainty(obj.counting_queue);
               }
           }
   
           obj.isCombined = false;
   
           // Suppression si faible certitude et objet bien observé
           if ((dist_to_obj >= 1.7 && dist_to_obj <= 3.0 && objectCoveredArea >= 0.8) &&
               obj.counting_queue.size() >= param_config.queue_thresh &&
               obj.exist_certainty < 0.25)
           {
               removeObject(id);
               ROS_INFO_STREAM(obj.name << " is removed !!");
           }
        }
    }
    void SemanticMap::addNewObject(const std::string &name, const float &confidence, const polygon &initial_shape, double &mean_height, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        SemanticObject obj;
        obj.name = name;
        obj.class_confidence = confidence;

        point shape_centroid;
        bg::centroid(initial_shape, shape_centroid);

        obj.shapes.push_back({initial_shape, shape_centroid, 1});
        obj.counting_queue.push(1);
        obj.isCombined = true;
        obj.exist_certainty = 1.0;
        obj.shape_union = initial_shape;
        obj.centroid_sum = shape_centroid;
        obj.centroid_sum_sq = point_square(shape_centroid);
        obj.centroid_mean = shape_centroid;
        obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
        obj.mean_height = mean_height;
        obj.point_cloud = cloud;
        bg::centroid(obj.shape_union, obj.shape_union_cen);

        // ANCIENNE méthode retirée (OBB rectangulaire)
        // double angle;
        // obj.oriented_box = create_oriented_box(obj.shape_union, angle);
        // obj.rotation_angle = angle;

        //  NOUVELLE méthode : appariement avec un modèle réel de la base
        std::pair<polygon, double> selected_model;
        selected_model = create_object_box_using_prior_knowledge(obj.shape_union, obj.name);
        obj.obb = selected_model.first;               // Ici c’est le contour polygonal du modèle connu aligné
        obj.obb_score = selected_model.second;        // Score d’association géométrique

        bg::centroid(obj.obb, obj.oriented_box_cen);  // On met à jour le centre de l’objet aligné

        addObject(obj);
    }


    void SemanticMap::addObject(const SemanticObject &obj)
    {
        if (obj.name == "Table"){
            tableList.insert(next_index);
        }
        //ROS_INFO_STREAM("Adding object" << obj.name << "to map");
        objectList[next_index] = obj;
        rtree_entry ent = {obj.bounding_box, next_index};
        objectRtree.insert(ent);
        //ROS_INFO_STREAM("Succesfully added, size: " << objectRtree.size());
        next_index++;
    }

    void SemanticMap::removeObject(size_t id)
    {
        SemanticObject &obj = objectList.at(id);
        objectRtree.remove(std::make_pair(obj.bounding_box, id));
        objectList.erase(id);
        if (tableList.find(id) != tableList.end()){
            tableList.erase(id);
        }
    }

    void SemanticMap::clearAll()
    {
        objectList.clear();
        objectRtree.clear();
        tableList.clear();
        next_index = 0;
    }

    point SemanticMap::getRobotPosition(){
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform;
        try{
            transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        point robot;
        robot.x(transform.transform.translation.x);
        robot.y(transform.transform.translation.y);
        return robot;
    }
    
    std::set<size_t> SemanticMap::getObjectsWithinRange(const polygon &pg)
    {
        double coveredAreaPercentage;
        std::vector<rtree_entry> result_obj;
        std::set<size_t> result;
        objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
        for (rtree_entry val : result_obj)
        {
            multi_polygon sect;
            const SemanticObject &foundObject = objectList.at(val.second);
            bg::intersection(pg, foundObject.shape_union, sect);
            coveredAreaPercentage = bg::area(sect)/bg::area(foundObject.shape_union);
            if(coveredAreaPercentage > 0.5)
                result.insert(val.second);
        }
        return result;
    }

    std::set<size_t> SemanticMap::getObjectsByNameInRange(const std::string &name, const polygon &pg)
    {
        std::vector<rtree_entry> result_obj;
        std::set<size_t> result;
        objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
        for (rtree_entry val : result_obj)
        {
            const SemanticObject &foundObject = objectList.at(val.second);
        
            if (foundObject.name == name && (bg::intersects(pg, foundObject.shape_union)
            || bg::within(pg, foundObject.shape_union) || bg::touches(pg, foundObject.shape_union)))
                result.insert(val.second);
            
        }
        return result;
    }

    std::set<size_t> SemanticMap::getObjectsInRange(const polygon &pg)
    {
        std::vector<rtree_entry> result_obj;
        std::set<size_t> result;
        objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
        for (rtree_entry val : result_obj)
        {
            const SemanticObject &foundObject = objectList.at(val.second);
            if (bg::intersects(pg, foundObject.shape_union) || bg::within(pg, foundObject.shape_union))
                result.insert(val.second);
        }
        return result;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr SemanticMap::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK (40);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);
        return cloud_filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr SemanticMap::removeRadiusOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.08);
        outrem.setMinNeighborsInRadius (1500);
        outrem.setKeepOrganized(true);
        // apply filter
        outrem.filter (*cloud_filtered);
        return cloud_filtered;
    }

    mapping_msgs::SemanticMap::Ptr SemanticMap::createMapMessage(const point &robot, double loaded)
    {
        bool isChair = false;
        bool isTable = false;
        int chairID = 0;
        int tableID = 0;


        ROS_WARN("erstelle map msg ");
        std::vector<int> remove_ids;

        mapping_msgs::SemanticMap::Ptr map(new mapping_msgs::SemanticMap);

        for (auto &val : objectList)
        {
            viewer_index += 1;
            SemanticObject &obj = val.second;
            if (obj.name == "Chair" && !isChair){
                isChair = true;
                chairID = val.first;
            }
            if (obj.name == "Table" && !isTable){
                isTable = true;
                tableID = val.first;
            }
            if(loaded == false) {
                if (bg::within(robot, obj.shape_union)) {
                    remove_ids.push_back(val.first);
                    ROS_INFO_STREAM("Object " << obj.name << " removed BECAUSE INTERFERENCE WITH ROBOT");
                    std::cout << "REMOVE OBJECT BECAUSE INTERFERENCE WITH ROBOT" << std::endl;
                    continue;
                }
            }
            if (obj.exist_certainty > 0.25)// param_config.certainty_thresh)
            {
               
                mapping_msgs::SemanticObject obj_msg;
                obj_msg.id = val.first;
                obj_msg.name = obj.name;
                obj_msg.exist_certainty = obj.exist_certainty;
                if(!obj.obb.outer().empty()){
                    obj_msg.obb = boostToPolygonMsg(obj.obb);
                }
                obj_msg.shape = boostToPolygonMsg(obj.shape_union);
                point centroid;
                bg::centroid(obj.shape_union, centroid);
                obj.shape_union_cen = centroid;
                obj_msg.position = boostToPointMsg(centroid);
                sensor_msgs::PointCloud2 msg_cloud;

                //ROS_WARN_STREAM("CREATING MAP MESSAGE for obj: "<< obj.name);
                if(obj.point_cloud != nullptr){
                    pcl::toROSMsg(*obj.point_cloud, msg_cloud);
                    msg_cloud.header.frame_id = "map";
                    obj_msg.pointcloud = msg_cloud;
                }

                map->objects.push_back(std::move(obj_msg));
            }
        }
    
        for(int i= 0; i<remove_ids.size(); i++){
            removeObject(remove_ids[i]);
        }
        map->header.frame_id = "map";
        map->header.stamp = ros::Time::now();
        return map;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr SemanticMap::createColoredPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double objectExistenceCertainty) 
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_cloud->header = cloud->header;

        // Default color (e.g., white) if class name not found
        uint8_t r = 255, g = 255, b = 255;

        // Find the color associated with the class name
        auto it = classColorMap.find(name);
        if (it != classColorMap.end()) {
            const std::tuple<uint8_t, uint8_t, uint8_t>& color = it->second;
            r = std::get<0>(color) * (1 - objectExistenceCertainty);
            g = std::get<1>(color) * (1 - objectExistenceCertainty);
            b = std::get<2>(color) * (1 - objectExistenceCertainty);
        }

        for (const auto& point : cloud->points) {
            pcl::PointXYZRGB color_point;
            color_point.x = point.x;
            color_point.y = point.y;
            color_point.z = point.z;
            color_point.r = r;
            color_point.g = g;
            color_point.b = b;
            color_cloud->points.push_back(color_point);
        }

        return color_cloud;
    }

    mapping_msgs::SemanticMap::Ptr SemanticMap::createMapMessage(const point &robot, double loaded, const ros::Publisher& poinCloudPublisher)
    {
        bool isChair = false;
        bool isTable = false;
        int chairID = 0;
        int tableID = 0;


        ROS_WARN("erstelle map msg ");
        std::vector<int> remove_ids;

        mapping_msgs::SemanticMap::Ptr map(new mapping_msgs::SemanticMap);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (auto &val : objectList)
        {
            viewer_index += 1;
            SemanticObject &obj = val.second;
            if (obj.name == "Chair" && !isChair){
                isChair = true;
                chairID = val.first;
            }
            if (obj.name == "Table" && !isTable){
                isTable = true;
                tableID = val.first;
            }
            if(loaded == false) {
                if (bg::within(robot, obj.shape_union)) {
                    remove_ids.push_back(val.first);
                    ROS_INFO_STREAM("Object " << obj.name << " removed BECAUSE INTERFERENCE WITH ROBOT");
                    std::cout << "REMOVE OBJECT BECAUSE INTERFERENCE WITH ROBOT" << std::endl;
                    continue;
                }
            }

            if (obj.exist_certainty > 0.25)// param_config.certainty_thresh)
            {
                mapping_msgs::SemanticObject obj_msg;
                obj_msg.id = val.first;
                obj_msg.name = obj.name;
                obj_msg.exist_certainty = obj.exist_certainty;
                if(!obj.obb.outer().empty()){
                    obj_msg.obb = boostToPolygonMsg(obj.obb);
                }
                obj_msg.shape = boostToPolygonMsg(obj.shape_union);
                point centroid;
                bg::centroid(obj.shape_union, centroid);
                obj.shape_union_cen = centroid;
                obj_msg.position = boostToPointMsg(centroid);
                sensor_msgs::PointCloud2 msg_cloud;

                //ROS_WARN_STREAM("CREATING MAP MESSAGE for obj: "<< obj.name);
                if(obj.point_cloud != nullptr){

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud = createColoredPointCloud(obj.name, obj.point_cloud, obj.exist_certainty);
                    *total_cloud += *color_cloud;

                    pcl::toROSMsg(*obj.point_cloud, msg_cloud);
                    msg_cloud.header.frame_id = "map";
                    obj_msg.pointcloud = msg_cloud;
                }

                map->objects.push_back(std::move(obj_msg));
            }
        }

        if (!total_cloud->empty()) {
            sensor_msgs::PointCloud2 msg_full_cloud;
            pcl::toROSMsg(*total_cloud, msg_full_cloud);
            msg_full_cloud.header.frame_id = "map";
            poinCloudPublisher.publish(msg_full_cloud);
        }

    
        for(int i= 0; i<remove_ids.size(); i++){
            removeObject(remove_ids[i]);
        }
        map->header.frame_id = "map";
        map->header.stamp = ros::Time::now();
        return map;
    }

    mapping_msgs::SemanticMap::Ptr SemanticMap::createGroundTruthMapMessage(){
        ROS_WARN("Ground Truth map msg ");
        mapping_msgs::SemanticMap::Ptr map(new mapping_msgs::SemanticMap);
        for (auto &val : groundTruthObjectList)
        {
            viewer_index += 1;
            SemanticObject &obj = val.second;
            mapping_msgs::SemanticObject obj_msg;
            obj_msg.id = val.first;
            obj_msg.name = obj.name;
            obj_msg.exist_certainty = 1;
            obj_msg.obb = boostToPolygonMsg(obj.shape_union);
            obj_msg.shape = boostToPolygonMsg(obj.shape_union);
            point centroid;
            bg::centroid(obj.shape_union, centroid);
            obj.shape_union_cen = centroid;
            obj_msg.position = boostToPointMsg(centroid);
            sensor_msgs::PointCloud2 msg_cloud;

            map->objects.push_back(std::move(obj_msg));
        }

        map->header.frame_id = "ground_truth_map";
        map->header.stamp = ros::Time::now();
        return map;
    }

    bool SemanticMap::writeLikelihoodData(std::ostream &output)
    {
        YAML::Node map;
        YAML::Node n;
        std::ostringstream sh;
        std::copy(likelihood_value_chair.begin(), likelihood_value_chair.end()-1, std::ostream_iterator<double>(sh, ","));
        sh << likelihood_value_chair.back();
        n["likelihood_chair"] = sh.str();
        std::ostringstream sh1;
        std::copy(time_value_chair.begin(), time_value_chair.end()-1, std::ostream_iterator<double>(sh1, ","));
        sh1 << time_value_chair.back();
        n["time_chair"] = sh1.str();
        std::ostringstream sh2;
        std::copy(likelihood_value_table.begin(), likelihood_value_table.end()-1, std::ostream_iterator<double>(sh2, ","));
        sh << likelihood_value_table.back();
        n["likelihood_table"] = sh2.str();
        std::ostringstream sh3;
        std::copy(time_value_table.begin(), time_value_table.end()-1, std::ostream_iterator<double>(sh3, ","));
        sh1 << time_value_table.back();
        n["time_table"] = sh3.str();
        map.push_back(n);
        output << map;
        return true;
    }

    bool SemanticMap::writeMapData(std::ostream &output)
    {
        YAML::Node map;
        for(const auto &map_entry : objectList)
        {
            const SemanticObject &obj = map_entry.second;
            YAML::Node n;
            n["name"] = obj.name;
            n["exist_certainty"] = obj.exist_certainty;
            n["oriented_box_rot"] = obj.rotation_angle;
            std::ostringstream sh;
            sh << bg::wkt(obj.shape_union);
            n["shape_union"] = sh.str();
            std::ostringstream sh_cen;
            sh_cen << bg::wkt(obj.shape_union_cen);
            n["shape_union_cen"] = sh_cen.str();
            std::ostringstream obb;
            obb << bg::wkt(obj.obb);
            n["oriented_box"] = obb.str();
            std::ostringstream obb_cen;
            obb_cen << bg::wkt(obj.oriented_box_cen);
            n["oriented_box_cen"] = obb_cen.str();
            n["oriented_box_score"] = obj.obb_score;
            map.push_back(n);
        }
        output << map;
        return true;
    }

    bool SemanticMap::readMapData(std::istream &input)
    {
        clearAll();
        YAML::Node map = YAML::Load(input);
        for (const YAML::Node &entry : map)
        {
            SemanticObject obj;
            obj.name = entry["name"].as<std::string>();
            obj.exist_certainty = entry["exist_certainty"].as<double>();
            try
            {
                bg::read_wkt(entry["shape_union"].as<std::string>(), obj.shape_union);
            }
            catch (const bg::read_wkt_exception &e)
            {
                ROS_ERROR_STREAM("Error reading object shape: " << e.what());
                return false;
            }
            try
            {
                bg::read_wkt(entry["shape_union_cen"].as<std::string>(), obj.shape_union_cen);
            }
            catch (const bg::read_wkt_exception &e)
            {
                ROS_ERROR_STREAM("Error reading object shape centroid: " << e.what());
                return false;
            }
            try
            {
                bg::read_wkt(entry["oriented_box"].as<std::string>(), obj.obb);
            }
            catch (const bg::read_wkt_exception &e)
            {
                ROS_ERROR_STREAM("Error reading obb: " << e.what());
                return false;
            }
            try
            {
                bg::read_wkt(entry["oriented_box_cen"].as<std::string>(), obj.oriented_box_cen);
            }
            catch (const bg::read_wkt_exception &e)
            {
                ROS_ERROR_STREAM("Error reading obb centroid: " << e.what());
                return false;
            }
            obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
            obj.obb_score = entry["oriented_box_score"].as<double>();
            addObject(obj);
        }
        cout << "Map to evaluate loaded !" << endl;
        return true;
    }
    
    
    bool SemanticMap::loadGroundTruthMap(std::istream &input)
    {
        groundTruthObjectList.clear();
        groundTruthObjectRtree.clear();
        groundTruthNext_index = 0;
        YAML::Node map = YAML::Load(input);
        for (const YAML::Node &entry : map)
        {
            SemanticObject obj;
            obj.name = entry["name"].as<std::string>();
            obj.id = entry["id"].as<int>();
            obj.exist_certainty = 1;
            try
            {
                bg::read_wkt(entry["shape_union"].as<std::string>(), obj.shape_union);
            }
            catch (const bg::read_wkt_exception &e)
            {
                ROS_ERROR_STREAM("Error reading object shape: " << e.what());
                return false;
            }
            obj.obb= obj.shape_union;
            obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
            groundTruthObjectList[groundTruthNext_index] = obj;
            rtree_entry ent = {obj.bounding_box, groundTruthNext_index};
            groundTruthObjectRtree.insert(ent);
            //ROS_INFO_STREAM("Succesfully added to GTMap, size: " << groundTruthObjectRtree.size());
            groundTruthNext_index++;
        }
        cout << "Ground Truth Map loaded !" << endl;
        return true;
    }
   
    mapping_msgs::ObjectPositions::Ptr SemanticMap::findObjectPosition(const mapping_msgs::FindObjects &request)
    {
        point robot = getRobotPosition();
        geometry_msgs::Point center;
        point near_to_position;
        near_to_position.x(request.near_to_position.x);
        near_to_position.y(request.near_to_position.y);
        mapping_msgs::ObjectPositions::Ptr position_msg(new mapping_msgs::ObjectPositions);
        if (request.near_to == "" && request.nearest_to_robot == false) {
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name) {
                    position_msg->name = obj.name;
                    center.x = obj.shape_union_cen.x();
                    center.y = obj.shape_union_cen.y();
                    position_msg->positions.push_back(center);
                }
            }
        }
        else if (request.nearest_to_robot == false) {
            std::vector<size_t> find_objects;
            std::vector<size_t> near_objects;
            size_t best_near_object;
            size_t best_find_object;
            for (auto &val : objectList) {
                const SemanticObject &obj = val.second;
                if (obj.name == request.name) {
                    find_objects.push_back(val.first);
                } 
                else if (obj.name == request.near_to) {
                    near_objects.push_back(val.first);
                }
            }
            double best_near_dist = std::numeric_limits<double>::infinity();
            for (size_t id : near_objects) {
                if (bg::distance(robot, objectList.at(id).shape_union_cen) < best_near_dist) {
                    best_near_object = id;
                    best_near_dist = bg::distance(robot, objectList.at(id).shape_union_cen);
                }
            }
            double best_find_dist = std::numeric_limits<double>::infinity();
            for (size_t id : find_objects) {
                if (bg::distance(objectList.at(best_near_object).shape_union_cen, objectList.at(id).shape_union_cen) < best_find_dist) {
                    best_find_object = id;
                    best_find_dist = bg::distance(objectList.at(best_near_object).shape_union_cen,
                                                objectList.at(id).shape_union_cen);
                }
            }
            position_msg->name = objectList.at(best_find_object).name;
            center.x = objectList.at(best_find_object).shape_union_cen.x();
            center.y = objectList.at(best_find_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }
        else if (request.nearest_to_robot == true){
            std::vector<size_t> find_objects;
            size_t best_object;
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name)
                    find_objects.push_back(val.first);
            }
            double best_dist = std::numeric_limits<double>::infinity();
            for(size_t id : find_objects){
                if (bg::distance(robot, objectList.at(id).shape_union_cen) < best_dist)
                    best_object = id;
            }
            position_msg->name = objectList.at(best_object).name;
            center.x = objectList.at(best_object).shape_union_cen.x();
            center.y = objectList.at(best_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }
        else if (bg::is_empty(near_to_position) == false){
            std::vector<size_t> find_objects;
            size_t best_object;
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name) {
                    find_objects.push_back(val.first);
                }
            }
            double best_dist = std::numeric_limits<double>::infinity();
            for(size_t id : find_objects){
                if (bg::distance(near_to_position, objectList.at(id).shape_union_cen) < best_dist)
                    best_object = id;
            }
            position_msg->name = objectList.at(best_object).name;
            center.x = objectList.at(best_object).shape_union_cen.x();
            center.y = objectList.at(best_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }
        return position_msg;
    }
    void SemanticMap::classRating(std::pair<std::string, std::array<double, 6>> &class_data,
        double mapping_factor,
        double com_offset)
    {
    std::array<double, 6> &data = class_data.second;

    data[0] += 1;  // ➤ Incrémente le nombre de True Positives (TP)

    // ➤ Moyenne glissante de l'IoU
    data[1] = ((data[1] * (data[0] - 1)) + mapping_factor) / data[0];

    // ➤ Moyenne glissante du décalage entre centroïdes
    data[2] = ((data[2] * (data[0] - 1)) + com_offset) / data[0];
    }



    void SemanticMap::save_stats(const std::vector<std::pair<std::string, std::array<double, 6>>> &all_classes_data,
        const std::string &filename)
    {
        std::ofstream myfile;
        myfile.open(filename, std::ios::out | std::ios::app);  // ➤ Ouvre en mode ajout
        myfile << "New map data\n";  // ➤ En-tête pour séparer les tests

        for (const auto &class_data : all_classes_data)
        {
        const std::string &name = class_data.first;
        const std::array<double, 6> &data = class_data.second;

        // ➤ Écrit : nom, TP, FP, Mean IOU, Mean Offset
        myfile << name << "," << data[0] << "," << data[3] << "," << data[1] << "," << data[2] << "\n";
        }

        myfile.close();
    }

    /*void SemanticMap::evaluteMap(std::string filename){
        std::vector<std::string> evaluted_classes{"Chair", "Table", "Shelf", "Couch","Skyscraper"};
        std::vector<std::pair<std::string, double*>> all_classes_data;
        double initial_data1[6]={0,0,0,0,0,0}; double initial_data2[6]={0,0,0,0,0,0}; double initial_data3[6]={0,0,0,0,0,0}; double initial_data4[6]={0,0,0,0,0,0}; double initial_data5[6]={0,0,0,0,0,0};
        all_classes_data.push_back(std::make_pair("Chair", initial_data1));
        all_classes_data.push_back(std::make_pair("Table", initial_data2));
        all_classes_data.push_back(std::make_pair("Shelf", initial_data3));
        all_classes_data.push_back(std::make_pair("Couch", initial_data4));
        all_classes_data.push_back(std::make_pair("Skyscraper", initial_data5));

        if(!objectList.empty()){
            // first: number of instances, second: our solution factor, third: dengler factor 
            int false_detection=0;
            for (auto &val : objectList){
                SemanticObject &obj = val.second;
                bool object_not_found= true;
                for(auto &val2 : groundTruthObjectList){
                    SemanticObject &gtObj = val2.second;
                    //cout << "object name " << obj.name <<" - Truth object name: "<< gtObj.name<< endl;
                    if(obj.exist_certainty > 0.25 && obj.name == gtObj.name){
                        double mapping_factor= iou(obj.obb, gtObj.obb);
                        double mapping_factor_dengler= iou(obj.shape_union, gtObj.obb);
                        point truth_centroid;
                        bg::centroid(gtObj.obb, truth_centroid);
                        double com_offset_dengler, com_offset;
                        com_offset_dengler= sqrt((obj.shape_union_cen.x() - truth_centroid.x())*(obj.shape_union_cen.x() - truth_centroid.x()) 
                                + (obj.shape_union_cen.y() - truth_centroid.y())*(obj.shape_union_cen.y() - truth_centroid.y()));
                        com_offset= sqrt((obj.oriented_box_cen.x() - truth_centroid.x())*(obj.oriented_box_cen.x() - truth_centroid.x()) 
                                + (obj.oriented_box_cen.y() - truth_centroid.y())*(obj.oriented_box_cen.y() - truth_centroid.y()));
                        
                        if(mapping_factor!=0)
                        {
                            //cout<< obj.name << val.first <<" represents object " << gtObj.name << val2.first << " with IOU: " << mapping_factor << " and CoM Offset: " 
                            //<< com_offset << endl;
                            object_not_found= false;
                            std::vector<std::pair<std::string, double*>>::iterator it;
                            for(it = all_classes_data.begin(); it != all_classes_data.end(); it++){
                                std::pair<std::string, double*> class_data = *it;
                                if(class_data.first == obj.name){
                                    classRating(class_data, mapping_factor, mapping_factor_dengler, com_offset, com_offset_dengler);
                                }
                            }     
                        }
                    }
                }

                if(obj.exist_certainty > 0.25 && object_not_found){
                    //cout << obj.name << val.first << " dont exist in the truth map!" << endl;
                    std::vector<std::pair<std::string, double*>>::iterator it;
                    for(it = all_classes_data.begin(); it != all_classes_data.end(); it++){
                        std::pair<std::string, double*> class_data = *it;
                        if(class_data.first == obj.name)
                            class_data.second[3]++;
                    }
                    false_detection++;
                }
            }

            cout <<"--- Map Evaluation ---" << endl;
            cout << std::left << setw(20)<< "class name" << setw(20)<< "TP" << setw(20)<< "FP"<< setw(20) << "Mean IOU" << setw(20) <<"Mean IOU Dengler"
            << setw(20) << "Mean CoM offset" << setw(20) <<"Mean CoM Dengler"<< endl;
            std::vector<std::pair<std::string, double*>>::iterator it;
            for(it = all_classes_data.begin(); it != all_classes_data.end(); it++){
                std::pair<std::string, double*> class_data = *it;
                cout << std::left << setw(20)<< class_data.first << setw(20) << class_data.second[0] << setw(20)<< class_data.second[3] << setw(20)
                << class_data.second[1] << setw(20) << class_data.second[4]<< setw(20) << class_data.second[2]<< setw(20) << class_data.second[5] << endl;
            }
            save_stats(all_classes_data, filename, true);
        }
        else
            ROS_INFO_STREAM("The Map is empty, so it can't be rated!");
    }*/
    void SemanticMap::evaluteMap(const std::string &filename)
    {
        // ➤ Liste des classes à évaluer
        std::vector<std::string> evaluted_classes{"Chair", "Table", "Shelf", "Couch"};

        // ➤ Initialisation des métriques : TP, Mean IoU, Mean CoM Offset, FP, _, _
        std::vector<std::pair<std::string, std::array<double, 6>>> all_classes_data;
        for (const auto &class_name : evaluted_classes)
            all_classes_data.emplace_back(class_name, std::array<double, 6>{0, 0, 0, 0, 0, 0});

        if (!objectList.empty())
        {
            for (auto &val : objectList)
            {
                SemanticObject &obj = val.second;
                bool object_not_found = true;

                for (auto &val2 : groundTruthObjectList)
                {
                    SemanticObject &gtObj = val2.second;

                    if (obj.exist_certainty > 0.25 && obj.name == gtObj.name)
                    {
                        bg::correct(obj.obb);
                        bg::correct(gtObj.obb); 
                        double mapping_factor = iou(obj.obb, gtObj.obb);
                        std::cout << "[DEBUG AREA] " << obj.name
                                  << " | Detected area: " << bg::area(obj.obb)
                                  << " | GT area: " << bg::area(gtObj.obb) << "\n";


                        point truth_centroid;
                        bg::centroid(gtObj.obb, truth_centroid);

                        double com_offset = std::hypot(obj.oriented_box_cen.x() - truth_centroid.x(),
                                                    obj.oriented_box_cen.y() - truth_centroid.y());
                        
                        std::cout << "[DEBUG] obj.name: " << obj.name
                                  << " | gtObj.name: " << gtObj.name
                                  << " | mapping_factor: " << mapping_factor << "\n";

                        if (mapping_factor != 0)
                        {
                            object_not_found = false;

                            for (auto &class_data : all_classes_data)
                            {
                                if (class_data.first == obj.name)
                                {
                                    classRating(class_data, mapping_factor, com_offset);
                                }
                            }
                        }
                    }
                }

                if (obj.exist_certainty > 0.25 && object_not_found)
                {
                    for (auto &class_data : all_classes_data)
                    {
                        if (class_data.first == obj.name)
                        {
                            class_data.second[3] += 1;  // ➤ Compte un faux positif
                        }
                    }
                }
            }

            // ➤ Affichage dans la console
            std::cout << "--- Map Evaluation ---\n";
            std::cout << std::left << std::setw(20) << "class name"
                    << std::setw(20) << "TP"
                    << std::setw(20) << "FP"
                    << std::setw(20) << "Mean IOU"
                    << std::setw(20) << "Mean CoM offset" << "\n";

            for (const auto &class_data : all_classes_data)
            {
                const auto &data = class_data.second;
                std::cout << std::left << std::setw(20) << class_data.first
                        << std::setw(20) << data[0]
                        << std::setw(20) << data[3]
                        << std::setw(20) << data[1]
                        << std::setw(20) << data[2] << "\n";
            }

            // ➤ Sauvegarde dans le fichier
            save_stats(all_classes_data, filename);
        }
        else
        {
            ROS_INFO_STREAM("The Map is empty, so it can't be rated!");
        }
    }


    std::map<size_t, SemanticObject> SemanticMap::getObjectList(){
        return objectList;
    }

    void SemanticMap::setObjectList(std::map<size_t, SemanticObject> object_List){
        clearAll();
        objectList= object_List;
    }

    size_t SemanticMap::getNextIndex(){
        return next_index;
    }
    
    void SemanticMap::setNextIndex(size_t nextIndex){
        next_index= nextIndex;
    }

    std::map<size_t, SemanticObject> SemanticMap::getGroundTruthObjectList(){
        return groundTruthObjectList;
    }

    void SemanticMap::setGroundTruthObjectList(std::map<size_t, SemanticObject> object_List){
        groundTruthObjectList.clear();
        groundTruthObjectList= object_List;
    }
}