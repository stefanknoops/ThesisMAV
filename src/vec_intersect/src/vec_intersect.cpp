/**
 * This file is part of the foe_estimator package - MAVLab TU Delft
 *
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 * */

#include <vec_intersect/vec_intersect.h>
#include <random>
#include <ros/console.h>
#include <numeric>

//#include "estimateFoECPP.h"

// Fill OF array with values
static std::vector<FlowPacket> fillOpticFlowArray()
{
  std::vector<FlowPacket> result;

  // Set the size of the array (COLUMN MAJOR)
  int OFbuff_size = final_buffer.size();

  int limit_vec = 10; // for limiting the total amount of vectors;
  int buf_idx = 0;

  // result.set_size(OFbuff_size, 4);

  prepMutex.lock();

  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < limit_vec; idx0++)
  {
    FlowPacket currentFlow = final_buffer[idx0];
    result.push_back(currentFlow);
  }

  final_buffer.clear();
  prepMutex.unlock();
  return result;
}

void estimateFoECPP(std::vector<FlowPacket> OpticFlow, double *FoE_x,
                    double *FoE_y)
{
  ROS_INFO("***STARTING NEW ESTIMATION***");

  std::random_device RandomSeed;
  std::mt19937 RandomNumber(RandomSeed());
  std::uniform_int_distribution<> RandomNumberDistribution(0, OpticFlow.size() - 1);

  int NumberOfIterations = 100;
  int max_inliers = 0;
  double threshold = 1;
  double FOEX, FOEY;

  int min_id = 0;
  int max_id = OpticFlow.size();

  std::vector<double> theta;
  std::vector<double> beta;

  std::vector<double> psi;
  std::vector<double> mag;
  ROS_INFO("loading OF");
  std::vector<double> d_x_all(OpticFlow.size()), d_y_all(OpticFlow.size()), d_theta_all(OpticFlow.size()), proj_d_all(OpticFlow.size()), p_toFoE(OpticFlow.size());

  for (int i = 0; i < OpticFlow.size(); i++)
  {

    double ang = atan2(OpticFlow[i].v, OpticFlow[i].u);
    theta.push_back(ang);
    beta.push_back(M_PI - ang);

    psi.push_back(M_PI - (M_PI - ang));

    mag.push_back(sqrt(OpticFlow[i].v * OpticFlow[i].v + OpticFlow[i].u * OpticFlow[i].u));
  }
  // ROS_INFO("start ransac");

  for (int n = 0; n < NumberOfIterations; n++)
  {

    int ind_1 = RandomNumberDistribution(RandomNumber);
    int ind_2 = RandomNumberDistribution(RandomNumber);

    if (!((abs(theta[ind_1] - theta[ind_2]) < 1e-8) || (abs(theta[ind_1] - theta[ind_2] - M_PI) < 1e-8) || (abs(theta[ind_1] - theta[ind_2] + M_PI) < 1e-8))) // check if not parallel
    {

      double d1 = OpticFlow[ind_1].x * sin(theta[ind_1]) + OpticFlow[ind_1].y * -cos(theta[ind_1]);
      double d2 = OpticFlow[ind_2].x * sin(theta[ind_2]) + OpticFlow[ind_2].y * -cos(theta[ind_2]);

      double a = sin(theta[ind_1]);
      double b = -cos(theta[ind_1]);
      double c = sin(theta[ind_2]);
      double d = -cos(theta[ind_2]);

      double det = (a * d - b * c);

      double a_inv = 1 / det * d;
      double b_inv = 1 / det * -b;
      double c_inv = 1 / det * -c;
      double d_inv = 1 / det * a;

      double FoE_test_X = a_inv * d1 + b_inv * d2;
      double FoE_test_Y = c_inv * d1 + d_inv * d2;

      std::cout << std::endl
                << std::endl;
      // std::cout << "Line 1 (x,y,u,v): " << OpticFlow[ind_1].x << ", " << OpticFlow[ind_1].y << ", " << OpticFlow[ind_1].u << ", " << OpticFlow[ind_1].v << std::endl;
      // std::cout << "Line 2 (x,y,u,v): " << OpticFlow[ind_2].x << ", " << OpticFlow[ind_2].y << ", " << OpticFlow[ind_2].u << ", " << OpticFlow[ind_2].v << std::endl;
      // std::cout << "Theta (1,2): " << theta[ind_1] << ", " << theta[ind_2] << std::endl;
      // std::cout << "d (1,2): " << d1 << ", " << d2 << std::endl;
      // std::cout << "Intersection: " << FoE_test_X << ", " << FoE_test_Y << std::endl;

      double d_x_1 = (FoE_test_X - OpticFlow[ind_1].x);
      double d_x_2 = (FoE_test_X - OpticFlow[ind_2].x);
      double d_y_1 = (FoE_test_Y - OpticFlow[ind_1].y);
      double d_y_2 = (FoE_test_Y - OpticFlow[ind_2].y);

      double d_theta_1 = abs(atan2(d_y_1, d_x_1) - theta[ind_1]);
      double d_theta_2 = abs(atan2(d_y_2, d_x_2) - theta[ind_2]);

      double proj_1 = mag[ind_1] * cos(d_theta_1);

      double proj_2 = mag[ind_2] * cos(d_theta_2);

      int inliers = 0;

      // std::cout << "proj (1,2): " << proj_1 << ", " << proj_2 << std::endl;

      if ((proj_1 <= 0) && (proj_2 <= 0))
      {
        // std::cout << "Divergent" << std::endl
        //           << std::endl
        //           << std::endl;

        for (int i = 0; i < OpticFlow.size(); i++)
        {
          d_x_all[i] = FoE_test_X - OpticFlow[i].x;
          d_y_all[i] = FoE_test_Y - OpticFlow[i].y;
          d_theta_all[i] = abs(atan2(d_y_all[i], d_x_all[i]) - theta[i]);

          proj_d_all[i] = mag[i] * cos(d_theta_all[i]);

          p_toFoE[i] = abs((FoE_test_X - OpticFlow[i].x) * sin(theta[i]) + (FoE_test_Y - OpticFlow[i].y) * -cos(theta[i]));

          // std::cout << "proj_d_all = " << proj_d_all[i] << ", p_toFoE = " << p_toFoE[i] << std::endl;

          if ((proj_d_all[i] < 0) && p_toFoE[i] < 250)
          {

            inliers++;
          }
        }
      }
      else
      {
        // std::cout << "Not divergent" << std::endl;
        //           << std::endl
        //           << std::endl;
      }
      // ROS_INFO("klaar");
      std::cout << "Inliers = " << inliers << ", n = " << n << std::endl;

      if (inliers > max_inliers)
      {
        FOEX = FoE_test_X;
        FOEY = FoE_test_Y;
        max_inliers = inliers;

        // std::cout << "new FoE (x,y) = " << FOEX << ", " << FOEY << std::endl;
      }
    }
  }

  if (FOEX != FOV_X / 2)
  {
    FoE_hist_x.push_back(FOEX);
  }
  if (FOEY != FOV_Y / 2)
  {
    FoE_hist_y.push_back(FOEY);
  }

  if (FoE_hist_x.size() > num_average)
  {
    FoE_hist_x.erase(FoE_hist_x.begin());
  }
  if (FoE_hist_y.size() > num_average)
  {
    FoE_hist_y.erase(FoE_hist_y.begin());
  }

  *FoE_x = std::accumulate(FoE_hist_x.begin(), FoE_hist_x.end(), 0.0) / FoE_hist_x.size();
  *FoE_y = std::accumulate(FoE_hist_y.begin(), FoE_hist_y.end(), 0.0) / FoE_hist_y.size();

  int64_t current_time = OpticFlow[0].t;

  double calctime = ros::WallTime::now().toSec() - beforetime;

  FoE_rec_file << current_time << ", " << *FoE_x << ", " << *FoE_y << ", " << FOEX << ", " << FOEY << ", " << FoE_hist_x.size() << ", " << OpticFlow.size() << ", " << max_inliers << ", " << "0 , " << calctime << std::endl;

//   for (int i = 0; i < OpticFlow.size(); i++)
//   {
//     // std::cout << current_time << "," << 0 << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << std::endl
//     // FoE_flow_rec_file << current_time << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << "," << OpticFlow[i].ru << "," << OpticFlow[i].rv << std::endl;
//   }
}

void estimationServer()
{
  // Fill final OF buffer to be used by FOE estimation
  log_OF(&myOF);
  // std::cout << "2" << std::endl;

  std::vector<FlowPacket> optic_flow;
  int buffersize = final_buffer.size();
  // ROS_INFO("estimationserver buffersize: %i", buffersize);
  // std::cout << "size: " << final_buffer.size() << std::endl;

  if (final_buffer.size() > min_vectors) // min amount of vectors required
  {

    // double beforetime = ros::Time::now().toSec();
    // std::cout << "3" << std::endl;

    // Run FOE estimation
    estimateFoECPP(final_buffer, &FoE_x, &FoE_y);
    // Add 0.5 to FoE_x for truncating by compiler to integer
    // double calctime = ros::Time::now().toSec() - beforetime;
    final_buffer.clear();

    // ROS_INFO("estimateFoECPP time: %f", calctime);

    // Publish the FOE onto its topic
  }
}

void opticflowCallback(const dvs_of_msg::FlowPacketMsgArray::ConstPtr &msg) // gets the optic flow from the
{

  beforetime = ros::WallTime::now().toSec();

  for (int i = 0; i < msg->flowpacketmsgs.size(); i = i + 1)
  {

    FlowPacket OFvec;
    OFvec.t = (uint64_t)(msg->flowpacketmsgs[i].t);

    // if (OFvec.t > 1e5) {
    OFvec.x = (int16_t)(msg->flowpacketmsgs[i].x);
    OFvec.y = (int16_t)(msg->flowpacketmsgs[i].y);
    OFvec.u = (float)(msg->flowpacketmsgs[i].u);
    OFvec.v = (float)(msg->flowpacketmsgs[i].v);
    OFvec.ru = (float)(msg->flowpacketmsgs[i].ru);
    OFvec.rv = (float)(msg->flowpacketmsgs[i].rv);
    myOF.push_back(OFvec);
    //}
  }

  // Run FOE estimation at rate_
  if (ros::Time::now().toNSec() - prev_time >= period_)
  {
    prev_time = ros::Time::now().toNSec();

    // Fill OF buffers and run FOE estimation

    // Run FOE estimation
    // std::cout << "1" << std::endl;

    estimationServer();

    FoE_msg.x = (int)FoE_x;
    FoE_msg.y = (int)FoE_y;
    FoE_pub.publish(FoE_msg);

    // Add 0.5 to FoE_x for truncating by compiler to integer
  }
}

void log_OF(std::vector<FlowPacket> *myOF)
{
  // Fill final_buffer with current OF vector and clear myOF for new OF from subscription.
  FlowPacket OFvec_buf;

  prepMutex.lock();
  for (std::vector<FlowPacket>::iterator it = myOF->begin(); it != myOF->end(); it++)

  {
    // double diff = last_ts - period_;
    // double currenttime = (*it).t;
    // if (currenttime >= diff)
    //{

    last_ts = (*it).t;
    // OFvec_buf.x = (*it).x;
    // OFvec_buf.y = (*it).y;
    // OFvec_buf.u = (*it).u;
    // OFvec_buf.v = (*it).v;
    // OFvec_buf.t = (*it).t;
    final_buffer.push_back((*it));
    //}
  }
  prepMutex.unlock();
  myOF->clear();
}

void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) // gets the optic flow from the
{

  OT_log_file << msg->header.stamp << ", " << msg->pose.position.x << ", " << msg->pose.position.x << ", " << msg->pose.position.x << ", ";
  OT_log_file << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w;

  OT_log_file << std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "foe_estimator");

  std::string folder;

  ros::NodeHandle n("~");

  n.getParam("folder", folder);
  ROS_INFO_STREAM("folder name: " << folder);
  int check = mkdir(folder.c_str(), 0777);

  // Initialize subscribers and publishers
  ros::Subscriber sub = n.subscribe("/OpticFlow", 1, opticflowCallback);
  ros::Subscriber sub2 = n.subscribe("/optitrack/pose", 1, optitrackCallback);

  FoE_pub = n.advertise<vec_intersect::FoE>("/FoE", 1);

  std::string myDate = currentDateTime();
  std::string filename = "FoE_recording.txt";
  FoE_rec_file.open(folder + filename);
  std::string filename_flow = "FoE_flow_recording.txt";
  FoE_flow_rec_file.open(folder + filename_flow);
  std::string filename_HP = "HP_recording.txt";
  HP_log_file.open(folder + filename_HP);
  std::string filename_HL = "HL_recording.txt";
  HL_log_file.open(folder + filename_HL);
  std::string filename_OT = "Optitrack_recording.txt";
  OT_log_file.open(folder + filename_OT);
  timelog.open(folder+"timing.txt");

  std::ofstream settings;
  settings.open(folder + "settings.txt");
  settings << "min_vectors = " << min_vectors << std::endl
           << "num_average = " << num_average << std::endl;
  settings << "AmountOfIterations = " << AmountOfIterations << std::endl
           << "MaxFailures = " << MaxFailures << std::endl;
  settings << "rate = " << rate_ << std::endl;
  settings.close();

  ros::spin();

  return 0;
}
