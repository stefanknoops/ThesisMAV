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

#include <cpp_foe/cpp_foe.h>
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
  // // FOR LIMITING THE AMOUNT OF FLOW VECTORS
  // int vec_limit = 10;
  // std::vector<FlowPacket> temp = OpticFlow;
  // OpticFlow.clear();
  // int factor = 1;

  // for (int i = 0; i < vec_limit; i++)
  // {
  //   // temp[i].u *=factor;
  //   // temp[i].v *=factor;
  //   OpticFlow.push_back(temp[i]);
  //   std::cout << "vector " << i << ": x=" << temp[i].x << ", y=" << temp[i].y << ", u=" << temp[i].u << ", v=" << temp[i].v << std::endl;
  //   // factor *= -1;
  // }

  int ArraySize = OpticFlow.size();
  // ROS_INFO("arraysize %i", ArraySize);

  double ResolutionX = 240.0;
  double ResolutionY = 180.0;

  std::vector<std::vector<double>> VectorArray(ArraySize, std::vector<double>{0.0, 0.0, 0.0});

  std::vector<std::vector<double>> NormalLines(ArraySize, std::vector<double>{0.0, 0.0, 0.0});
  std::vector<std::vector<double>> all_rules(ArraySize, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  std::vector<std::vector<double>> InitialHullLines = {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, -ResolutionY}, {1.0, 0.0, -ResolutionX}};
  std::vector<std::vector<double>> InitialHullRules = {{0.0, 1.0, 0.0, -1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, -1.0, 0.0}, {0.0, 1.0, -ResolutionY, 1.0, 0.0, 0.0}, {1.0, 0.0, -ResolutionX, 0.0, 1.0, 0.0}};
  std::vector<std::vector<double>> InitialHullPoints = {{0.0, 0.0}, {0.0, ResolutionY}, {ResolutionX, ResolutionY}, {ResolutionX, 0.0}};
  std::vector<std::vector<double>> BestHull = InitialHullPoints;
  std::vector<std::vector<double>> BestHullLines = InitialHullLines;

  std::vector<std::vector<double>> HullLines;
  std::vector<std::vector<double>> HullRules;
  std::vector<std::vector<double>> HullPoints = InitialHullPoints;

  bool NewHull = false;
  int MaxScore = 0;
  int MinScore = 10000;
  int MaxUsed = 0;

  bool Illegal;

  std::random_device RandomSeed;
  std::mt19937 RandomNumber(RandomSeed());
  std::uniform_int_distribution<> RandomNumberDistribution(0, ArraySize - 1);

  //  Set method parameters
  //  Set # iterations for RANSAC search
  //  Set # failures allowed in iteration before stopping search
  //  Initialize frame boundaries to hull
  //  Rules: [A, B, C, sign(v), sign(u)]
  //  Initialize optical flow variables
  //  Write into ax + by + c = 0 ==> perp equation = bx-ay + lambda= 0
  // ROS_INFO("New iteration");
  for (int i = 0; i < ArraySize; i++)
  {

    double A = OpticFlow[i].u;
    double B = OpticFlow[i].v;

    double C = -(A * OpticFlow[i].x + B * OpticFlow[i].y);

    double D = -(B * OpticFlow[i].x + -A * OpticFlow[i].y);

    // NormalLines[i][2] = (OpticFlow[i].x * (OpticFlow[i].y + OpticFlow[i].v) - (OpticFlow[i].x + OpticFlow[i].u) * OpticFlow[i].y);
    // ROS_INFO("Vector = %lf x + %lf y + %lf = 0",A,B,C);
    // ROS_INFO("Normal = %lf x + %lf y + %lf = 0",B,-A,D);
    VectorArray[i][0] = B;
    VectorArray[i][1] = -A;
    VectorArray[i][2] = D;
    NormalLines[i][0] = A;
    NormalLines[i][1] = B;

    NormalLines[i][2] = C;

    // std::cout << "OF vector = " << A << "\t" << B << "\t" << C << std::endl;
    // std::cout << "Normal vector = " << B << "\t" << -A << "\t" << D << std::endl;
  }
  //  Create a list with the rules for all vectors
  //  Rule: [A, B, C, sign(v), sign(u)] // not sure what rules are
  //  Put line descriptions in same format for halfplane check (change sign if negative)

  for (int i = 0; i < ArraySize; i++)
  {
    double sign_u = signum(OpticFlow[i].u); // source: google cpp signum
    double sign_v = signum(OpticFlow[i].v); // source: google cpp signum
    if (NormalLines[i][0] < 0)
    {
      all_rules[i] = std::vector<double>{-NormalLines[i][0], -NormalLines[i][1], -NormalLines[i][2], sign_v, sign_u};
    }
    else
    {
      all_rules[i] = std::vector<double>{NormalLines[i][0], NormalLines[i][1], NormalLines[i][2], sign_v, sign_u};
    }
  }

  // add initial hull to all lines


  for (int n = 0; n < AmountOfIterations; n++)
  {
    // for n iterations
    // ROS_INFO("iteration %i", n);
    //    initiate hull
    std::vector<int> VectorInHull(NormalLines.size(), 0);
    std::vector<int> VectorTried(NormalLines.size(), 0);

    HullLines = InitialHullLines;
    HullRules = InitialHullRules;
    HullPoints = InitialHullPoints;

    bool StopSearch = false;
    int StopCount = 0;
    std::vector<double> RandomNormalRule;
    int AmountOfVectorsUsed = 0;
    // set all vectors to not tried

    while (!StopSearch) // while not stopping search
    {                   // take random vector and check whether it has not been used yet

      int RandomNormalIndex = RandomNumberDistribution(RandomNumber);
      if (std::all_of(VectorTried.begin(), VectorTried.end(), [](int x)
                      { return x == 1; }))
      {
        StopSearch = true;
        // ROS_INFO("STOP 1");
      }
      else
      {

        while ((VectorTried[RandomNormalIndex] == 1 || VectorInHull[RandomNormalIndex] == 1) && StopSearch == false)
        {
          RandomNormalIndex = RandomNumberDistribution(RandomNumber);
        };
      };

      if (StopSearch)
      {
        break;
      }

      // ROS_INFO("Random vector index = %i", RandomNormalIndex);

      VectorTried[RandomNormalIndex] = 1;
      AmountOfVectorsUsed += 1;
      // see if all vectors are tried yet

      // choose vector
      std::vector<double> RandomNormal = NormalLines[RandomNormalIndex];
      // std::cout << "Normal = " << RandomNormal[0] << "\t" << RandomNormal[1] << "\t" << RandomNormal[2] << std::endl;

      double sign_u = signum(OpticFlow[RandomNormalIndex].u); // source: google cpp signum
      double sign_v = signum(OpticFlow[RandomNormalIndex].v); // source: google cpp signum

      // put vector in standard format, add index
      if (RandomNormal[0] < 0.0)
      {

        RandomNormalRule = {-RandomNormal[0], -RandomNormal[1], -RandomNormal[2], sign_v, sign_u, (double)RandomNormalIndex};
      }
      else
      {
        RandomNormalRule = {RandomNormal[0], RandomNormal[1], RandomNormal[2], sign_v, sign_u, (double)RandomNormalIndex};
      }

      bool AddCheck = false;
      int IllegalCount = 0;

      for (int i = 0; i < HullLines.size(); i++) // for all hull lines
      {
        // if not parallel
        // std::vector<double> MergeMatrixC = {-HullLines[i][2], -RandomNormal[2]};

        double CramerA = HullLines[i][0];
        double CramerB = HullLines[i][1];
        double CramerE = -HullLines[i][2];

        double CramerC = RandomNormal[0];
        double CramerD = RandomNormal[1];
        double CramerF = -RandomNormal[2];

   

        double DeterminantA = CramerA * CramerD - CramerB * CramerC;

        if (DeterminantA != 0.0)
        {
          // calculate intersection
          // x = (c1b2 - c2b1)/determinant
          // y = (a1c2 - a2c1)/determinant

          double IntersectionX = (CramerE * CramerD - CramerB * CramerF) / DeterminantA;
          double IntersectionY = (CramerA * CramerF - CramerC * CramerE) / DeterminantA;



          Illegal = false;
          for (int j = 0; j < HullRules.size(); j++) // for all rules
          {
            /* code */
            // check whether vector does not violate rules
            std::vector<double> CurrentRule = HullRules[j];

            if ((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1))
            {

              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];
              // ROS_INFO("intersect1 = %lf", IntersectionCalculation);

              if (IntersectionCalculation > FLT_EPSILON)
              {
                // ROS_INFO("illegal 1 ");
                Illegal = true;
                IllegalCount += 1;
              }
            }
            else if ((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1))
            {

              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];

              if (IntersectionCalculation < -FLT_EPSILON)
              {

                Illegal = true;
                IllegalCount += 1;
              }
            }
          }
          // if not violated, add to hull
          if (!Illegal && !AddCheck)
          {
            HullLines.push_back(RandomNormal);
            int EndOfVector = HullLines.size() - 1;
            HullRules.push_back(RandomNormalRule);
            VectorInHull[RandomNormalIndex] = 1;
            AddCheck = true;
          }
          if (!Illegal)
          {
            std::vector<double> IntersectionVector = {IntersectionX, IntersectionY};
            HullPoints.push_back(IntersectionVector);
            AddCheck = true;
          }

          if (HullRules.size() == IllegalCount)
          {
            StopCount += 1;
            if (StopCount == MaxFailures)
            {
              StopSearch = true;
            }
          }

          // if not contributing, dismiss vector
        }

        else
        {
          IllegalCount += 1;
        }
      }

      std::vector<int> PointsToRemove;
      for (int i = 0; i < HullPoints.size(); i++) // for all hull points
      {

        // check latest rule
        double CurrentPointX = HullPoints[i][0];
        double CurrentPointY = HullPoints[i][1];
        Illegal = false;

        std::vector<double> CurrentRule = HullRules.back();

        if (((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1)) && !Illegal)
        {
          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation > FLT_EPSILON)
          {

            PointsToRemove.push_back(i);
            Illegal = true;
          }
        }
        else if (((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1)) && !Illegal)
        {

          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation < -FLT_EPSILON)
          {

            Illegal = true;
            PointsToRemove.push_back(i);
          }
        }
      }
      for (int i = PointsToRemove.size() - 1; i >= 0; i--) // from back to front to avoid changing the index halfway
      {

        HullPoints.erase(HullPoints.begin() + PointsToRemove[i]);
      }

      std::vector<int> LinesToRemove;
      double RuleCheck = 0;
      std::vector<double> RuleCheckVector;
      int RuleCheckSum = 0;
      for (int j = 0; j < HullLines.size(); j++)
      {
        for (int k = 0; k < HullPoints.size(); k++)
        {
          RuleCheck = HullRules[j][0] * HullPoints[k][0] + HullRules[j][1] * HullPoints[k][1] + HullRules[j][2];
          RuleCheckVector.push_back(RuleCheck);
        }

        for (int l = 0; l < RuleCheckVector.size(); l++)
        {
          if (abs(RuleCheckVector[l]) < FLT_EPSILON)
          {
            RuleCheckSum += 1;
          }
        }
        if (RuleCheckSum == 0)
        {
          LinesToRemove.push_back(j);
          VectorInHull[HullRules[j][5]] = 0;
        }
      }

      if (LinesToRemove.size() != HullRules.size())
      {
        for (int i = LinesToRemove.size() - 1; i >= 0; i--) // from back to front to avoid changing the index halfway
        {
          HullLines.erase(HullLines.begin() + LinesToRemove[i]);
          HullRules.erase(HullRules.begin() + LinesToRemove[i]);
        }
      }


    }

    double IterationX = 0.0;
    double IterationY = 0.0;
    int InlierCount = 0;
    int IterationScore = 0;
    for (int i = 0; i < HullPoints.size(); i++)
    {
      IterationX = IterationX + HullPoints[i][0];
      IterationY = IterationY + HullPoints[i][1];
    }
    IterationX = IterationX / (double)HullPoints.size();
    IterationY = IterationY / (double)HullPoints.size();

    for (int i = 0; i < all_rules.size(); i++)
    {
      std::vector<double> CurrentRule = all_rules[i];
      if ((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1))
      {
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation < FLT_EPSILON)
        {

          InlierCount += 1;
        }
      }
      else if ((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && signum(CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1))
      {
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation > -FLT_EPSILON)
        {
          InlierCount += 1;
        }
      }
    }

    IterationScore = InlierCount;



    if ((IterationScore > MaxScore) || ((IterationScore == MaxScore) && (AmountOfVectorsUsed > MaxUsed)))
    {
      NewHull = true;
      MaxScore = IterationScore;
      BestHull = HullPoints;
      BestHullLines = HullLines;
      MaxUsed = AmountOfVectorsUsed;
      ROS_INFO("Found new hull with score %i using %i, in iteration %i", IterationScore, AmountOfVectorsUsed, n);
    }
  }

  if (NewHull)
  {
    double FOEX = 0;
    double FOEY = 0;
    for (int i = 0; i < BestHull.size(); i++)
    {
      
      FOEX = (FOEX + BestHull[i][0]);
      FOEY = (FOEY + BestHull[i][1]);

    }

    FOEX = FOEX / (double)BestHull.size();
    FOEY = FOEY / (double)BestHull.size();

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

    FoE_rec_file << current_time << ", " << *FoE_x << ", " << *FoE_y << ", " << FOEX << ", " << FOEY << ", " << FoE_hist_x.size() << ", " << ArraySize << ", " << MaxScore << ", " << MaxUsed << ", " << calctime << std::endl;

    // for (int i = 0; i < BestHullLines.size(); i++)
    // { // std::cout << "besthull" << current_time << "," << BestHullLines[i][0] << "," << BestHullLines[i][1] << "," << BestHullLines[i][2] << std::endl;

    //   HL_log_file << current_time << "," << BestHullLines[i][0] << "," << BestHullLines[i][1] << "," << BestHullLines[i][2] << std::endl;
    // }

    // for (int i = 0; i < BestHull.size(); i++)
    // {
    //   HP_log_file << current_time << "," << BestHull[i][0] << "," << BestHull[i][1] << std::endl;
    // }

    // for (int i = 0; i < ArraySize; i++)
    // {
    //   // std::cout << current_time << "," << 0 << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << std::endl
    // //   FoE_flow_rec_file << current_time << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << "," << OpticFlow[i].ru << "," << OpticFlow[i].rv << std::endl;
    // }
  }
}

void estimationServer()
{
  // Fill final OF buffer to be used by FOE estimation
  log_OF(&myOF);

  std::vector<FlowPacket> optic_flow;
  int buffersize = final_buffer.size();

  if (final_buffer.size() > min_vectors) // min amount of vectors required
  {


    // Run FOE estimation
    estimateFoECPP(final_buffer, &FoE_x, &FoE_y);

    final_buffer.clear();

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
    estimationServer();

    FoE_msg.x = (int)FoE_x;
    FoE_msg.y = (int)FoE_y;
    FoE_pub.publish(FoE_msg);

  }
}

void log_OF(std::vector<FlowPacket> *myOF)
{
  // Fill final_buffer with current OF vector and clear myOF for new OF from subscription.
  FlowPacket OFvec_buf;

  prepMutex.lock();
  for (std::vector<FlowPacket>::iterator it = myOF->begin(); it != myOF->end(); it++)

  {

    last_ts = (*it).t;
    final_buffer.push_back((*it));
    //}
  }
  prepMutex.unlock();
  myOF->clear();
}

void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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

  FoE_pub = n.advertise<cpp_foe::FoE>("/FoE", 1);

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
  timelog.open(folder + "timing.txt");

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
