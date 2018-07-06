#ifndef PR2_MOTORS_ANALYZER_H
#define PR2_MOTORS_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace diagnostic_aggregator {

class PR2MotorsAnalyzer : public Analyzer
{
public:
  PR2MotorsAnalyzer();

  ~PR2MotorsAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:

  // Store status item for EtherCAT master
  boost::shared_ptr<StatusItem> eth_master_item_;

  std::string path_, nice_name_, power_board_name_;

  bool runstop_hit_, has_initialized_, has_power_data_, has_eth_data_;
};

}
#endif //PR2_MOTORS_ANALYZER_H
