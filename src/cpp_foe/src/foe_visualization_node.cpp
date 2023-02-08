#include "foe_visualization.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "foe_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  foe_visualization::FoeVisualization foe_visualization(nh, nh_private);

  ros::spin();

  return 0;
}
