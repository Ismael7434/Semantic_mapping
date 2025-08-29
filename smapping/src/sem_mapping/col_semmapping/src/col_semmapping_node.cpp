#include <iostream>
#include <ros/ros.h>
#include <col_semmapping/sem_fusion.h>
#include <semanticmap.h>
#include <csignal>
#include <Eigen/Dense>
#include <chrono>
using namespace Eigen;

ros::Publisher refMapPub;
ros::Publisher filteredRefMapPub;
ros::Publisher recMapPub;
ros::Publisher filteredRecMapPub;
ros::Publisher globalMapPub;
ros::Publisher waitingObjPub;

// Testing parameters
std::string algorithm = "our_solution"; // our_solution or modified_nms
double fusion_overlap_threshold = 0.1;
std::string buffer = "0.1";
std::string reference_map_name = "ref_map_test2"; // ref_cluttered good map with false orientations & ref_map2 very bad_map & ref_map3 good map
std::string reference_map_file_name = "src/sem_mapping/col_semmapping/fused_maps/reference_maps/" + reference_map_name + ".yaml";
std::string fused_map_file_name = "src/sem_mapping/col_semmapping/fused_maps/fused_maps/test_fusion.yaml";

// validation
std::string ground_truth_map_file_name = "src/sem_mapping/col_semmapping/fused_maps/ground_truth_maps/truth_map_cluttered_world_2.yaml"; // validation environment
std::string single_robot_maps_directory = "/home/isma/smapping/src/sem_mapping/col_semmapping/fused_maps/single_robot_maps/last_version_maps";
std::string global_map_directory = "/home/isma/smapping/src/sem_mapping/col_semmapping/fused_maps/fused_maps/global_map";
std::string fused_maps_directory = "/home/isma/smapping/src/sem_mapping/col_semmapping/fused_maps/fused_maps/global_map";
std::string nms_fused_maps_directory = "/home/isma/smapping/src/sem_mapping/col_semmapping/fused_maps/fused_maps/nms_maps";
std::string modified_nms_fused_maps_directory = "/home/isma/smapping/src/sem_mapping/col_semmapping/fused_maps/fused_maps/modified_nms_maps";
std::string received_map_file_name = "src/sem_mapping/col_semmapping/fused_maps/single_robot_maps/validation_maps/val7.yaml";

std::string prior_knowledge_file_name = "/home/isma/smapping/src/sem_mapping/col_semmapping/cfg/prior knowledge.yaml";

// Indicators saving files
// std::string backup_file_name= "/home/abdessalem/smapping/src/sem_mapping/col_semmapping/statistical_data/validation_data/fusion evaluation/" + algorithm + "/threshold_0.1.csv"; //"_nms"+".csv";
//std::string backup_file_name = "/home/abdessalem/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/inputMaps.csv";
//std::string backup_file_name = "/home/abdessalem/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/progressive_fusion_updated.csv";
std::string backup_file_name = "/home/isma/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/iterations _0_10_20_test2.csv";

void printAvailableCommands()
{
  cout << "--------- Available commands -------------" << endl;
  cout << "1) fuse_maps" << endl;
  cout << "2) fuse_all_maps_in_directory" << endl;
  cout << "3) evaluate_reference_map" << endl;
  cout << "4) evaluate_cleared_reference_map" << endl;
  cout << "5) evaluate_received_map" << endl;
  cout << "6) evaluate_fused_map" << endl;
  cout << "7) evaluate_all_maps_in_directory" << endl;
  cout << "8) f1Score_all_maps_in_directory" << endl;
  cout << "9) f1Score_all_cleared_maps" << endl;
  cout << "10) evaluate_cleared_received_map" << endl;
  cout << "11) nms_fuse_maps" << endl;
  cout << "12) nms_fuse_all_maps_in_directory" << endl;
  cout << "13) modified_nms_fuse_maps" << endl;
  cout << "14) modified_nms_fuse_all_maps_in_directory" << endl;
  cout << "15) progressive_fusion" << endl;
}

int get_command_number(std::string command)
{
  if (command == "fuse_maps")
    return 1;
  else if (command == "evaluate_reference_map")
    return 2;
  else if (command == "evaluate_received_map")
    return 3;
  else if (command == "evaluate_fused_map")
    return 4;
  else if (command == "fuse_all_maps_in_directory")
    return 5;
  else if (command == "evaluate_all_maps_in_directory")
    return 6;
  else if (command == "evaluate_cleared_reference_map")
    return 7;
  else if (command == "f1Score_all_maps_in_directory")
    return 8;
  else if (command == "f1Score_all_cleared_maps")
    return 9;
  else if (command == "evaluate_cleared_received_map")
    return 10;
  else if (command == "nms_fuse_maps")
    return 11;
  else if (command == "nms_fuse_all_maps_in_directory")
    return 12;
  else if (command == "modified_nms_fuse_maps")
    return 13;
  else if (command == "modified_nms_fuse_all_maps_in_directory")
    return 14;
  else if (command == "progressive_fusion")
    return 15;
  else
    return 20;
}

std::string readNext(const std::string &str, std::string::const_iterator &begin)
{
  std::string::const_iterator end = str.cend();
  std::string res;
  res.reserve(str.size());

  for (; begin != end && *begin == ' '; begin++)
    ; // skip leading whitespaces

  char end_char = ' ';
  if (begin != end && *begin == '"')
  {
    end_char = '"';
    begin++;
  }

  bool escape = false;
  for (; begin != end; begin++)
  {
    if (escape)
      escape = false;
    else if (*begin == '\\')
    {
      escape = true;
      continue;
    }
    else if (*begin == end_char)
    {
      begin++; // remove end character
      break;
    }
    res += *begin;
  }
  return res;
}

void sigintHandler(int sig)
{
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "col_semmapping_node");
  ros::NodeHandle n;

  // Initialize maps publishers
  filteredRefMapPub = n.advertise<mapping_msgs::SemanticMap>("/filtered_reference_map", 1, true);
  refMapPub = n.advertise<mapping_msgs::SemanticMap>("/original_reference_map", 1, true);
  filteredRecMapPub = n.advertise<mapping_msgs::SemanticMap>("/filtered_received_map", 1, true);
  recMapPub = n.advertise<mapping_msgs::SemanticMap>("/original_received_map", 1, true);
  globalMapPub = n.advertise<mapping_msgs::SemanticMap>("/fused_map", 1, true);
  waitingObjPub = n.advertise<mapping_msgs::SemanticMap>("/waiting_objects", 1, true);

  semmapping::SemanticFusion fusion_node;
  fusion_node.show_map_id();

  // Acquisition of prior knowledge
  std::ifstream prior_knowledge_file(prior_knowledge_file_name);
  fusion_node.setPriorKnowledge(prior_knowledge_file);
  prior_knowledge_file.close();

  // Definition of input/output maps
  std::ifstream ref_map_file(reference_map_file_name);
  std::ifstream received_map_file(received_map_file_name);
  std::ofstream fusion_file(fused_map_file_name);

  // Definition of all input/output maps
  tf2_ros::Buffer tfBuffer(ros::Duration(20));
  pcl::visualization::PCLVisualizer *viewer;
  boost::mutex viewer_mtx;
  int vport0 = 0;
  int vport1 = 1;
  int vport2 = 2;
  int vport3 = 3;
  int semmap_vport0 = 4;
  int semmap_vport1 = 5;
  semmapping::ParamsConfig param_config;

  semmapping::SemanticMap ref_map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);
  semmapping::SemanticMap filtered_ref_map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);
  semmapping::SemanticMap received_map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);
  semmapping::SemanticMap filtered_received_map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);
  semmapping::SemanticMap global_map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);
  semmapping::SemanticMap waiting_objects(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);

  signal(SIGINT, sigintHandler);

  // Read reference map and create cleared reference map
  ref_map.readMapData(ref_map_file);
  ref_map_file.close();
  fusion_node.removeMapInconsistencies(ref_map, filtered_ref_map);
  std::ifstream file(ground_truth_map_file_name);
  ref_map.loadGroundTruthMap(file);
  std::map<size_t, semmapping::SemanticObject> groundTruthObjectList = ref_map.getGroundTruthObjectList();

  // Create and publish reference (original/cleared) maps messages
  semmapping::point robot;
  mapping_msgs::SemanticMap::Ptr refMapMsg = ref_map.createMapMessage(robot, true);
  refMapPub.publish(refMapMsg);
  mapping_msgs::SemanticMap::Ptr filteredRefMapMsg = filtered_ref_map.createMapMessage(robot, true);
  filteredRefMapPub.publish(filteredRefMapMsg);
  cout << "-- Original & Cleared reference maps are published --" << endl;

  // Create and publish received (original/cleared) maps messages
  received_map.readMapData(received_map_file);
  received_map_file.close();
  mapping_msgs::SemanticMap::Ptr recMapMsg = received_map.createMapMessage(robot, true);
  recMapPub.publish(recMapMsg);

  fusion_node.removeMapInconsistencies(received_map, filtered_received_map);
  mapping_msgs::SemanticMap::Ptr filteredRecMapMsg = filtered_received_map.createMapMessage(robot, true);
  filteredRecMapPub.publish(filteredRecMapMsg);

  cout << "-- Original & Cleared received maps are published --" << endl;

  while (1)
  {
    printAvailableCommands();
    std::string in;
    std::getline(std::cin, in);
    std::string::const_iterator it = in.cbegin();
    std::string command = readNext(in, it);
    int command_number = get_command_number(command);

    switch (command_number)
    {
    case 1: // fuse_maps
    {
      // Fuse and save the two maps
      fusion_node.semfusion_updated(filtered_ref_map, filtered_received_map, global_map, fusion_overlap_threshold); // fusion_node.semfusion(ref_map, received_map, global_map);
      global_map.writeMapData(fusion_file);
      fusion_file.close();
      // Create and publish fused map message
      mapping_msgs::SemanticMap::Ptr fused_map_msg = global_map.createMapMessage(robot, true);
      globalMapPub.publish(fused_map_msg);
      cout << "-- Fused map is published --" << endl;
      break;
    }
    case 2: // evaluate_reference_map
    {
      std::ifstream file(ground_truth_map_file_name);
      ref_map.loadGroundTruthMap(file);
      file.close();
      // Evaluate reference map
      fusion_node.evaluateSemanticMap(ref_map.getObjectList(), ref_map.getGroundTruthObjectList(), 
        "/home/isma/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/referenceMap1.csv");
      break;
    }
    case 3: // evaluate_received_map
    {
      std::ifstream file2(ground_truth_map_file_name);
      received_map.loadGroundTruthMap(file2);
      received_map.evaluteMap(backup_file_name);
      int false_negative = fusion_node.numberFalseNegativeInMap(received_map.getObjectList(), received_map.getGroundTruthObjectList());
      std::pair<int, int> positive_detection = fusion_node.numberTrueFalseDetectionInMap(received_map.getObjectList(), received_map.getGroundTruthObjectList());
      int true_positive = positive_detection.first;
      int false_positive = positive_detection.second;
      double precision, recall, f1Score;
      std::array<double, 3> indicators;
      indicators = fusion_node.computeMapF1Score(true_positive, false_positive, false_negative);
      precision = indicators[0];
      recall = indicators[1];
      f1Score = indicators[2];
      cout << "FN= " << false_negative << " TP= " << true_positive << " FP= " << false_positive << endl;
      cout << "precision= " << precision << " recall= " << recall << " f1Score= " << f1Score << endl;
      file2.close();
      break;
    }
    case 4: // evaluate_fused_map
    {
      std::ifstream ground_truth_map_file(ground_truth_map_file_name);
      global_map.loadGroundTruthMap(ground_truth_map_file);
      fusion_node.evaluateSemanticMap(global_map.getObjectList(), global_map.getGroundTruthObjectList(), backup_file_name);
      break;
    }
    case 5: // fuse_all_maps_in_directory
    {
      int filecount = 0;
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        filecount++;
        std::ifstream received_map_file(dir_entry.path().string());
        received_map.readMapData(received_map_file);
        received_map_file.close();
        // Filter the map
        filtered_received_map.clearAll();
        fusion_node.removeMapInconsistencies(received_map, filtered_received_map);
        // Fuse the two maps
        global_map.clearAll();
        fusion_node.semfusion_updated(filtered_ref_map, filtered_received_map, global_map, fusion_overlap_threshold); // fusion_node.semfusion(ref_map, received_map, global_map);
        // Save the fused map
        std::stringstream filename;
        filename << fused_maps_directory + "/map_" << std::setw(4) << std::setfill('0') << filecount << ".yaml";
        std::ofstream fusion_file(filename.str().c_str());
        global_map.writeMapData(fusion_file);
        fusion_file.close();
      }
      break;
    }
    case 6: // evaluate_all_maps_in_directory
    {
      std::ifstream ground_truth_map_file(ground_truth_map_file_name);
      global_map.loadGroundTruthMap(ground_truth_map_file);

      std::map<std::string, semmapping::ClassStats> all_classes_stats;
      // Populate all_classes_stats with class names and initialize to zero
      all_classes_stats["Chair"] = semmapping::ClassStats();
      all_classes_stats["Table"] = semmapping::ClassStats();
      all_classes_stats["Shelf"] = semmapping::ClassStats();
      all_classes_stats["Couch"] = semmapping::ClassStats();
      int numberObjectsWithOBB = 0;
      int mapID = 0;

      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        std::ifstream fused_map_file(dir_entry.path().string());
        global_map.readMapData(fused_map_file);
        fused_map_file.close();
        mapID++;
        fusion_node.evaluateObjectCategoriesOverAllMaps(mapID, all_classes_stats, numberObjectsWithOBB, global_map.getObjectList(), global_map.getGroundTruthObjectList(), backup_file_name);
      }
      fusion_node.saveMapStats(all_classes_stats, backup_file_name);
      break;
    }
    case 7: // evaluate_cleared_reference_map
    {
      std::ifstream file(ground_truth_map_file_name);
      filtered_ref_map.loadGroundTruthMap(file);
      file.close();
      // Evaluate reference map
      fusion_node.evaluateSemanticMap(filtered_ref_map.getObjectList(), filtered_ref_map.getGroundTruthObjectList(), 
        "/home/isma/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/referenceMap1.csv");
      break;
    }
    case 8: // f1Score_all_maps_in_directory
    {
      std::ifstream ground_truth_map_file(ground_truth_map_file_name);
      received_map.loadGroundTruthMap(ground_truth_map_file);
      ground_truth_map_file.close();
      int index = 0;
      double meanF1Score = 0;
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        std::ifstream map_file(dir_entry.path().string());
        received_map.readMapData(map_file);
        map_file.close();
        int false_negative = fusion_node.numberFalseNegativeInMap(received_map.getObjectList(), received_map.getGroundTruthObjectList());
        std::pair<int, int> positive_detection = fusion_node.numberTrueFalseDetectionInMap(received_map.getObjectList(), received_map.getGroundTruthObjectList());
        int true_positive = positive_detection.first;
        int false_positive = positive_detection.second;
        double precision, recall, f1Score;
        std::array<double, 3> indicators;
        indicators = fusion_node.computeMapF1Score(true_positive, false_positive, false_negative);
        precision = indicators[0];
        recall = indicators[1];
        f1Score = indicators[2];
        index++;
        meanF1Score += f1Score;
        cout << "Map " << index << endl;
        cout << "FN= " << false_negative << " TP= " << true_positive << " FP= " << false_positive << endl;
        cout << "precision= " << precision << " recall= " << recall << " f1Score= " << f1Score << endl;
      }
      meanF1Score /= index;
      cout << "meanF1Score= " << meanF1Score << endl;
      break;
    }
    case 9: // f1Score_all_cleared_maps
    {
      int index = 0;
      double meanF1Score = 0;
      double meanF1ScoreBefore = 0;
      std::ofstream myfile;
      double overlap_threshold = 0.0;
      std::ostringstream s;
      s << overlap_threshold;
      std::string filteringStastsFilename = "src/sem_mapping/col_semmapping/statistical_data/filtering evaluation/filtering_threshold_" + s.str() + ".csv";
      myfile.open(filteringStastsFilename, ios::out | ios::app);
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        std::ifstream map_file(dir_entry.path().string());
        received_map.readMapData(map_file);
        map_file.close();
        filtered_received_map.clearAll();
        int tp, fp, fn;
        std::pair<int, int> positive_detection2;
        double precision, recall, f1Score;
        std::array<double, 3> indicators2;

        fn = fusion_node.numberFalseNegativeInMap(received_map.getObjectList(), groundTruthObjectList);
        positive_detection2 = fusion_node.numberTrueFalseDetectionInMap(received_map.getObjectList(), groundTruthObjectList);
        tp = positive_detection2.first;
        fp = positive_detection2.second;
        indicators2 = fusion_node.computeMapF1Score(tp, fp, fn);
        precision = indicators2[0];
        recall = indicators2[1];
        f1Score = indicators2[2];

        meanF1ScoreBefore += f1Score;

        cout << "Map " << index << endl;
        cout << "FN= " << fn << " TP= " << tp << " FP= " << fp << endl;
        cout << "precision= " << precision << " recall= " << recall << " f1Score= " << f1Score << endl;

        myfile << "MAP " << index << "\n";
        myfile << tp << "," << fp << "," << fn << "," << precision << "," << recall << "," << f1Score << "\n";

        fusion_node.removeMapInconsistencies(received_map, filtered_received_map, overlap_threshold);

        fn = fusion_node.numberFalseNegativeInMap(filtered_received_map.getObjectList(), groundTruthObjectList);
        positive_detection2 = fusion_node.numberTrueFalseDetectionInMap(filtered_received_map.getObjectList(), groundTruthObjectList);
        tp = positive_detection2.first;
        fp = positive_detection2.second;
        indicators2 = fusion_node.computeMapF1Score(tp, fp, fn);
        precision = indicators2[0];
        recall = indicators2[1];
        f1Score = indicators2[2];

        index++;
        meanF1Score += f1Score;
        cout << "Map " << index << endl;
        cout << "FN= " << fn << " TP= " << tp << " FP= " << fp << endl;
        cout << "precision= " << precision << " recall= " << recall << " f1Score= " << f1Score << endl;

        myfile << tp << "," << fp << "," << fn << "," << precision << "," << recall << "," << f1Score << "\n";
      }
      meanF1Score /= index;
      meanF1ScoreBefore /= index;
      cout << "meanF1ScoreBefore= " << meanF1ScoreBefore << "meanF1ScoreAfter= " << meanF1Score << endl;
      myfile << meanF1ScoreBefore << "," << meanF1Score << "\n";
      myfile.close();
      break;
    }
    case 10: // evaluate_cleared_received_map
    {
      std::ifstream file(ground_truth_map_file_name);
      filtered_received_map.loadGroundTruthMap(file);
      filtered_received_map.evaluteMap(backup_file_name);
      int false_negative = fusion_node.numberFalseNegativeInMap(filtered_received_map.getObjectList(), filtered_received_map.getGroundTruthObjectList());
      std::pair<int, int> positive_detection = fusion_node.numberTrueFalseDetectionInMap(filtered_received_map.getObjectList(), filtered_received_map.getGroundTruthObjectList());
      int true_positive = positive_detection.first;
      int false_positive = positive_detection.second;
      std::array<double, 3> indicators;
      indicators = fusion_node.computeMapF1Score(true_positive, false_positive, false_negative);
      double precision, recall, f1Score;
      precision = indicators[0];
      recall = indicators[1];
      f1Score = indicators[2];
      cout << "FN= " << false_negative << " TP= " << true_positive << " FP= " << false_positive << endl;
      cout << "precision= " << precision << " recall= " << recall << " f1Score= " << f1Score << endl;
      file.close();
      break;
    }
    case 11: // nms_fuse_maps
    {
      // Fuse and save the two maps
      fusion_node.semfusion_nms(filtered_ref_map, received_map, global_map, 0.5); // fusion_node.semfusion(ref_map, received_map, global_map);
      global_map.writeMapData(fusion_file);
      fusion_file.close();
      // Create and publish fused map message
      mapping_msgs::SemanticMap::Ptr fused_map_msg = global_map.createMapMessage(robot, true);
      globalMapPub.publish(fused_map_msg);
      cout << "-- NMS Fused map is published --" << endl;
      break;
    }
    case 12: // nms_fuse_all_maps_in_directory
    {
      int filecount = 0;
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        filecount++;
        std::ifstream received_map_file(dir_entry.path().string());
        received_map.readMapData(received_map_file);
        received_map_file.close();
        // Fuse the two maps
        global_map.clearAll();
        fusion_node.semfusion_nms(filtered_ref_map, received_map, global_map, 0.5);
        // Save the fused map
        std::stringstream filename;
        filename << nms_fused_maps_directory + "/map_" << std::setw(4) << std::setfill('0') << filecount << ".yaml";
        std::ofstream fusion_file(filename.str().c_str());
        global_map.writeMapData(fusion_file);
        fusion_file.close();
      }
      break;
    }
    case 13: // modified_nms_fuse_maps
    {
      // Fuse and save the two maps
      fusion_node.semfusion_modified_nms(filtered_ref_map, received_map, global_map, 0.5); // fusion_node.semfusion(ref_map, received_map, global_map);
      global_map.writeMapData(fusion_file);
      fusion_file.close();
      // Create and publish fused map message
      mapping_msgs::SemanticMap::Ptr fused_map_msg = global_map.createMapMessage(robot, true);
      globalMapPub.publish(fused_map_msg);
      cout << "-- Modified NMS Fused map is published --" << endl;
      break;
    }
    case 14: // modified_nms_fuse_all_maps_in_directory
    {
      int filecount = 0;
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        filecount++;
        std::ifstream received_map_file(dir_entry.path().string());
        received_map.readMapData(received_map_file);
        received_map_file.close();
        // Filter the map
        filtered_received_map.clearAll();
        fusion_node.removeMapInconsistencies(received_map, filtered_received_map);
        // Fuse the two maps
        global_map.clearAll();
        fusion_node.semfusion_modified_nms(filtered_ref_map, filtered_received_map, global_map, 0.5);
        // Save the fused map
        std::stringstream filename;
        filename << modified_nms_fused_maps_directory + "/map_" << std::setw(4) << std::setfill('0') << filecount << ".yaml";
        std::ofstream fusion_file(filename.str().c_str());
        global_map.writeMapData(fusion_file);
        fusion_file.close();
      }
      break;
    }
    case 15: // progressive_fusion
    {
      MatrixXd P = MatrixXd::Identity(8, 8); // Adjust the initial uncertainty as needed
      // std::cout << "P= " << P(0,0) << ", " << P(1,1) << ", " << P(2,2) << ", " << P(3,3) << ", ... , " << P(7,7) << endl;

      // Define process noise covariance matrix
      float a = 0.01;
      MatrixXd Q(8, 8);
      Q << a, 0, 0, 0, 0, 0, 0, 0,
          0, a, 0, 0, 0, 0, 0, 0,
          0, 0, a, 0, 0, 0, 0, 0,
          0, 0, 0, a, 0, 0, 0, 0,
          0, 0, 0, 0, a, 0, 0, 0,
          0, 0, 0, 0, 0, a, 0, 0,
          0, 0, 0, 0, 0, 0, a, 0,
          0, 0, 0, 0, 0, 0, 0, a;

      // Define measurement noise covariance matrix
      float b = 0.2; // 0.03
      MatrixXd R(8, 8);
      R << b, 0, 0, 0, 0, 0, 0, 0,
          0, b, 0, 0, 0, 0, 0, 0,
          0, 0, b, 0, 0, 0, 0, 0,
          0, 0, 0, b, 0, 0, 0, 0,
          0, 0, 0, 0, b, 0, 0, 0,
          0, 0, 0, 0, 0, b, 0, 0,
          0, 0, 0, 0, 0, 0, b, 0,
          0, 0, 0, 0, 0, 0, 0, b;

      fusion_node.initializeCategoryObjectNumber();
      fusion_node.countObjectNumberPerCategoryInMap(filtered_ref_map);

      // Load ground truth map
      std::ifstream ground_truth_map_file(ground_truth_map_file_name);
      global_map.loadGroundTruthMap(ground_truth_map_file);
      
      //fusion_node.evaluateSemanticMap(filtered_ref_map.getObjectList(), global_map.getGroundTruthObjectList(), backup_file_name);

      waiting_objects.clearAll();

      int maps_number = 0;
      for (const auto &dir_entry : boost::filesystem::directory_iterator(single_robot_maps_directory))
      {
        maps_number++;
      }

      // Object to track intial state
      std::string trackBackupFileName = "/home/isma/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/objectTracking.csv";
      std::string objectToTrackName = "Chair";
      int objectId = 4;
      // wasMapped, wasAdded, wasUpdated, wasRemoved, isInGlobalMap, isInWaitingList, notInMap, existenceProbability
      semmapping::ObjectState state = {false, false, false, false, true, false, false, 0.0};
      cout << "Object initial state : wasMapped = " << state.wasMapped << " wasAdded = " << state.wasAdded << " wasUpdated = " << state.wasUpdated << " wasRemoved = " << state.wasRemoved << " isInGlobalMap = " << state.isInGlobalMap << " isInWaitingList = " << state.isInWaitingList << " notInMap = " << state.notInMap << " existenceProbability = " << state.existenceProbability << endl;

      cout << "The folder contain " << maps_number << " maps to integrate in the global map" << endl;

      // for (const auto & dir_entry: boost::filesystem::directory_iterator(single_robot_maps_directory)){
      for (int filecount = 1; filecount <= maps_number; filecount++)
      {
        std::cin.get();

        /* Launch chrono to compute fusion time */
        //std::chrono::time_point<std::chrono::system_clock> start, end;
        //start = std::chrono::system_clock::now();

        // Acquisition of prior knowledge
        std::ifstream prior_knowledge_file(prior_knowledge_file_name);
        fusion_node.updatePriorKnowledge(prior_knowledge_file);
        prior_knowledge_file.close();

        std::ostringstream map_index;
        map_index << filecount;
        std::string new_map_path = single_robot_maps_directory + "/map" + map_index.str() + ".yaml";
        // std::cout<<new_map_path<<endl;

        // Publish the last cleared refernce map
        mapping_msgs::SemanticMap::Ptr filteredRefMapMsg = filtered_ref_map.createMapMessage(robot, true);
        filteredRefMapPub.publish(filteredRefMapMsg);

        // Get and publish new received map before processing
        std::ifstream received_map_file(new_map_path);
        received_map.clearAll();
        received_map.readMapData(received_map_file);
        received_map_file.close();
        mapping_msgs::SemanticMap::Ptr recMapMsg = received_map.createMapMessage(robot, true);
        recMapPub.publish(recMapMsg);

        // Evaluate recieved map
        /*fusion_node.evaluateSemanticMap(received_map.getObjectList(), global_map.getGroundTruthObjectList(), backup_file_name);*/

        // Filter and publish the filtered received map
        filtered_received_map.clearAll();
        fusion_node.removeMapInconsistencies(received_map, filtered_received_map);
        mapping_msgs::SemanticMap::Ptr filteredRecMapMsg = filtered_received_map.createMapMessage(robot, true);
        filteredRecMapPub.publish(filteredRecMapMsg);

        // Fuse the two maps
        global_map.clearAll();
        fusion_node.globalMapUpdate(filtered_ref_map, filtered_received_map, global_map, waiting_objects, P, Q, R, fusion_overlap_threshold);

        // Visualize the global map and waiting objects map
        mapping_msgs::SemanticMap::Ptr fused_map_msg = global_map.createMapMessage(robot, true);
        globalMapPub.publish(fused_map_msg);
        cout << "Global map updated " << filecount << " time(s)" << endl;
        mapping_msgs::SemanticMap::Ptr waiting_objects_msg = waiting_objects.createMapMessage(robot, true);
        waitingObjPub.publish(waiting_objects_msg);
        cout << "Waiting objects updated " << filecount << " time(s)" << endl;

        // Evaluate Global map
        //fusion_node.evaluateSemanticMap(global_map.getObjectList(), global_map.getGroundTruthObjectList(), backup_file_name);

        // Track object state
        
        /*fusion_node.trackGtObjectState(objectToTrackName, objectId, global_map.getGroundTruthObjectList(), filtered_received_map.getObjectList(),
        global_map.getObjectList(), waiting_objects.getObjectList(), state ,trackBackupFileName);
        cout << "Object state at iteration" << filecount << " : wasMapped = " << state.wasMapped << " wasAdded = " << state.wasAdded
          << " wasUpdated = " << state.wasUpdated << " wasRemoved = " << state.wasRemoved << " isInGlobalMap = " << state.isInGlobalMap 
          << " isInWaitingList = " << state.isInWaitingList << " notInMap = " << state.notInMap << " existenceProbability = " << state.existenceProbability << endl;*/
        

        // Update the cleared reference map
        filtered_ref_map.clearAll();
        for (auto &value : global_map.getObjectList())
        {
          semmapping::SemanticObject &obj = value.second;
          filtered_ref_map.addObject(obj);
        }

        // Save the fused map
        /*std::stringstream filename;
        filename << global_map_directory +"/map_after_" << std::setw(4) << std::setfill('0') << filecount << "_fusion"<< ".yaml";
        std::ofstream global_map_file(filename.str().c_str());
        global_map.writeMapData(global_map_file);
        global_map_file.close();*/

        /* Computing the fusion time */
        /*end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        cout<<"Fusion iteration number : "<< filecount <<"     Fusion time = "<< elapsed_seconds.count() <<endl;
        std::ofstream myfile;
        myfile.open ("/home/abdessalem/smapping/src/sem_mapping/col_semmapping/statistical_data/progressive_fusion/fusion time.csv", ios::out | ios::app);
        myfile << "Fusion : " << filecount << " , " << "Time : " << elapsed_seconds.count() << "\n";
        myfile.close();*/
      }
      break;
    }
    default:
      std::cout << "Command \"" << command << "\" not recognized" << std::endl;
    }
  }
}