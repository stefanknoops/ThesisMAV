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

//#include "estimateFoECPP.h"

// Fill OF array with values
static std::vector<FlowPacket> fillOpticFlowArray()
{
  std::vector<FlowPacket> result;

  // Set the size of the array (COLUMN MAJOR)
  int OFbuff_size = final_buffer.size();
  int buf_idx = 0;

  // result.set_size(OFbuff_size, 4);

  prepMutex.lock();

  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < OFbuff_size; idx0++)
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
  int ArraySize = OpticFlow.size();
  // ROS_INFO("arraysize %i", ArraySize);

  double ResolutionX = 240.0;
  double ResolutionY = 180.0;

  std::vector<std::vector<double>> norm_lines(ArraySize, std::vector<double>{0.0, 0.0, 0.0});
  std::vector<std::vector<double>> all_rules(ArraySize, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  std::vector<std::vector<double>> InitialHullLines = {{0.0, 1.0, -1.0}, {1.0, 0.0, -1.0}, {0.0, 1.0, -ResolutionY}, {1.0, 0.0, -ResolutionX}};
  std::vector<std::vector<double>> InitialHullRules = {{0.0, 1.0, -1.0, -1.0, 0.0, 0.0}, {1.0, 0.0, -1.0, 0.0, -1.0, 0.0}, {0.0, 1.0, -ResolutionY, 1.0, 0.0, 0.0}, {1.0, 0.0, -ResolutionX, 0.0, 1.0, 0.0}};
  std::vector<std::vector<double>> InitialHullPoints = {{1.0, 1.0}, {1.0, ResolutionY}, {ResolutionX, ResolutionY}, {ResolutionX, 1.0}};
  std::vector<std::vector<double>> BestHull = InitialHullPoints;
  std::vector<std::vector<double>> HullLines;
  std::vector<std::vector<double>> HullRules;
  std::vector<std::vector<double>> HullPoints = InitialHullPoints;
  int MaxScore = 0;
  bool Illegal;

  int AmountOfIterations = 100;

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

  for (int i = 0; i < ArraySize; i++)
  {

    double A = OpticFlow[i].u;
    double B = OpticFlow[i].v;

    double C = -(A * OpticFlow[i].x + B * OpticFlow[i].y);
    norm_lines[i][0] = B;
    norm_lines[i][1] = -A;

    // norm_lines[i][2] = (OpticFlow[i].x * (OpticFlow[i].y + OpticFlow[i].v) - (OpticFlow[i].x + OpticFlow[i].u) * OpticFlow[i].y);
    norm_lines[i][2] = C;

    // ROS_INFO("vector = %lf u + %lf v @ x = %lf and y = %lf", OpticFlow[i].u,OpticFlow[i].v,(double)OpticFlow[i].x,(double)OpticFlow[i].y);
    // ROS_INFO("Norm line = %lf x + %lf y + %lf", B,-A,C);
  }

  //  Create a list with the rules for all vectors
  //  Rule: [A, B, C, sign(v), sign(u)] // not sure what rules are
  //  Put line descriptions in same format for halfplane check (change sign if negative)

  for (int i = 0; i < ArraySize; i++)
  {
    double sign_v = signum(OpticFlow[i].v); // source: google cpp signum
    double sign_u = signum(OpticFlow[i].u); // source: google cpp signum
    if (norm_lines[i][0] < 0)
    {
      all_rules[i] = std::vector<double>{-norm_lines[i][0], -norm_lines[i][1], -norm_lines[i][2], sign_v, sign_u};
    }
    else
    {
      all_rules[i] = std::vector<double>{norm_lines[i][0], norm_lines[i][1], norm_lines[i][2], sign_v, sign_u};
    }
  }

  // add initial hull to all lines

  // all_lines = norm_lines;
  // for (i = 0; i < InitialHullLines.size(); i++)
  // {
  //   std::vector<double> append_vector = {InitialHullLines[i]};
  //   all_lines.push_back(append_vector);
  // }

  // AmountOfIterations = std::min(AmountOfIterations,ArraySize);
  int MaxFailures = 1;

  for (int n = 0; n < AmountOfIterations; n++)
  { // for n iterations
    // ROS_INFO("iteration %i", n);
    //   initiate hull
    std::vector<int> VectorInHull(norm_lines.size(), 0);
    std::vector<int> VectorTried(norm_lines.size(), 0);

    HullLines = InitialHullLines;
    HullRules = InitialHullRules;
    HullPoints = InitialHullPoints;

    bool StopSearch = false;
    int StopCount = 0;
    std::vector<double> RandomVectorRule;

    // set all vectors to not tried

    while (!StopSearch) // while not stopping search
    {                   // take random vector and check whether it has not been used yet

      int RandomVectorIndex = RandomNumberDistribution(RandomNumber);
      if (std::all_of(VectorTried.begin(), VectorTried.end(), [](int x)
                      { return x == 1; }))
      {
        StopSearch = true;
        // ROS_INFO("STOP");
      }
      else
      {

        while ((VectorTried[RandomVectorIndex] == 1 || VectorInHull[RandomVectorIndex] == 1) && StopSearch == false)
        {
          RandomVectorIndex = RandomNumberDistribution(RandomNumber);
        };
      };

      if (StopSearch)
      {
        break;
      }

      // ROS_INFO("Random vector index = %i", RandomVectorIndex);

      VectorTried[RandomVectorIndex] = 1;

      // see if all vectors are tried yet

      // choose vector
      std::vector<double> RandomVector = norm_lines[RandomVectorIndex];

      double sign_u = signum(OpticFlow[RandomVectorIndex].u); // source: google cpp signum

      double sign_v = signum(OpticFlow[RandomVectorIndex].v); // source: google cpp signum
      // put vector in standard format, add index
      if (RandomVector[0] < 0.0)
      {
        RandomVector[0] = -RandomVector[0];
        RandomVector[1] = -RandomVector[1];
        RandomVector[2] = -RandomVector[2];

        RandomVectorRule = {RandomVector[0], RandomVector[1], RandomVector[2], sign_v, sign_u, (double)RandomVectorIndex};
      }
      else
      {
        RandomVectorRule = {RandomVector[0], RandomVector[1], RandomVector[2], sign_v, sign_u, (double)RandomVectorIndex};
      }

      bool AddCheck = false;
      int IllegalCount = 0;

      for (int i = 0; i < HullLines.size(); i++) // for all hull lines
      {
        // if not parallel
        // std::vector<double> MergeMatrixC = {-HullLines[i][2], -RandomVector[2]};

        double CramerA = HullLines[i][0];
        double CramerB = HullLines[i][1];
        double CramerE = -HullLines[i][2];

        double CramerC = RandomVector[0];
        double CramerD = RandomVector[1];
        double CramerF = -RandomVector[2];

        // ROS_INFO("HL a %lf", HullLines[i][0]);
        // ROS_INFO("HL b %lf", HullLines[i][1]);
        // ROS_INFO("HL c %lf", HullLines[i][2]);

        double DeterminantA = CramerA * CramerD - CramerB * CramerC;

        if (DeterminantA != 0.0)
        {
          // calculate intersection
          // x = (c1b2 - c2b1)/determinant
          // y = (a1c2 - a2c1)/determinant

          double IntersectionX = (CramerE * CramerD - CramerB * CramerF) / DeterminantA;
          double IntersectionY = (CramerA * CramerF - CramerC * CramerE) / DeterminantA;

          // ROS_INFO("x %lf", IntersectionX);
          // ROS_INFO("y %lf", IntersectionY);

          Illegal = false;
          for (int j = 0; j < HullRules.size(); j++) // for all rules
          {
            // ROS_INFO("i,j: %i %i",i,j);
            /* code */
            // check whether vector does not violate rules
            std::vector<double> CurrentRule = HullRules[j];

            if ((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1))
            {

              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];
              // ROS_INFO("intersect1 = %lf", IntersectionCalculation);

              if (IntersectionCalculation > 0.0)
              {
                // ROS_INFO("illegal");
                Illegal = true;
                IllegalCount += 1;
              }
            }
            else if ((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1))
            {

              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];
              // ROS_INFO("intersect2 = %lf", IntersectionCalculation);

              if (IntersectionCalculation < 0.0)
              {
                // ROS_INFO("illegal");

                Illegal = true;
                IllegalCount += 1;
              }
            }
          }
          // if not violated, add to hull
          if (!Illegal && !AddCheck)
          {
            HullLines.push_back(RandomVector);
            HullRules.push_back(RandomVectorRule);
            VectorInHull[RandomVectorIndex] = 1;
            AddCheck = true;
          }
          // ROS_INFO("line 258 illegal? %i", Illegal);
          if (!Illegal)
          {
            std::vector<double> IntersectionVector = {IntersectionX, IntersectionY};
            HullPoints.push_back(IntersectionVector);
            // ROS_INFO("Added a point");
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
          // ROS_INFO("Parallel");
        }
      }

      std::vector<int> PointsToRemove;
      for (int i = 0; i < HullPoints.size(); i++) // for all hull points
      {

        /* code */
        // check latest rule
        double CurrentPointX = HullPoints[i][0];
        double CurrentPointY = HullPoints[i][1];
        Illegal = false;

        std::vector<double> CurrentRule = HullRules.back();

        if (((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1)) && !Illegal)
        {
          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation > 0.0)
          {

            PointsToRemove.push_back(i);
            Illegal = true;
            // ROS_INFO("pushed back: %i", i);
          }
        }
        else if (((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1)) && !Illegal)
        {

          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation < 0.0)
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
          if (RuleCheckVector[l] == 0)
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
    // remove lines and rules

    // calculate score
    double IterationX = 0.0;
    double IterationY = 0.0;
    int InlierCount = 0;
    int IterationScore = 0;
    for (int i = 0; i < HullPoints.size(); i++)
    {
      IterationX = IterationX + HullPoints[i][0];
      IterationY = IterationY + HullPoints[i][1];
      // ROS_INFO("hp x %lf y %lf",HullPoints[i][0],HullPoints[i][1]);
    }
    IterationX = IterationX / (double)HullPoints.size();
    IterationY = IterationY / (double)HullPoints.size();

    for (int i = 0; i < all_rules.size(); i++)
    {
      std::vector<double> CurrentRule = all_rules[i];
      if ((signum(CurrentRule[1]) == 1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == 1))
      {
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation <= 0.0)
        {

          InlierCount += 1;
        }
      }
      else if ((signum(CurrentRule[1]) == -1 && (CurrentRule[3]) == 1) || (signum(CurrentRule[1]) == 1 && signum(CurrentRule[3]) == -1) || (signum(CurrentRule[1]) == 0 && (CurrentRule[4]) == -1) || (signum(CurrentRule[0]) == 0 && (CurrentRule[3]) == -1))
      {
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation >= 0.0)
        {
          InlierCount += 1;
        }
      }
    }
    // ROS_INFO("inlier score %i", InlierCount);

    IterationScore = InlierCount;
    // ROS_INFO("it score %i", IterationScore);
    //  ROS_INFO("max score %i", MaxScore);

    if (IterationScore > MaxScore)
    {
      MaxScore = IterationScore;
      BestHull = HullPoints;
    }
  }

  double FOEX = 0;
  double FOEY = 0;
  for (int i = 0; i < BestHull.size(); i++)
  {
    // ROS_INFO("point %i: %lf, %lf", i, BestHull[i][0], BestHull[i][1]);
    FOEX = FOEX + BestHull[i][0];
    FOEY = FOEY + BestHull[i][1];
    ////ROS_INFO("FOEX %lf", FOEX);
    // ROS_INFO("FOEY %lf", FOEY);
  }

  *FoE_x = FOEX / (double)BestHull.size();
  *FoE_y = FOEY / (double)BestHull.size();

  // ROS_INFO("test2 %i", BestHull.size());

  // ROS_INFO("test %lf", test);

  // ROS_INFO("FOEX %lf", FOEX);
  // ROS_INFO("FOEX %lf", *FoE_x);
  // ROS_INFO("FOEY %lf", FOEY);
  // ROS_INFO("FOEY %lf", *FoE_y);
}

void estimationServer()
{
  // Fill final OF buffer to be used by FOE estimation
  log_OF(&myOF);
  std::vector<FlowPacket> optic_flow;
  int buffersize = final_buffer.size();
  // ROS_INFO("estimationserver buffersize: %i", buffersize);
  if (final_buffer.size() > 5)
  {

    optic_flow = fillOpticFlowArray();

    // double beforetime = ros::Time::now().toSec();

    // Run FOE estimation
    estimateFoECPP(optic_flow, &FoE_x, &FoE_y);
    // Add 0.5 to FoE_x for truncating by compiler to integer
    // double calctime = ros::Time::now().toSec() - beforetime;

    // ROS_INFO("estimateFoECPP time: %f", calctime);

    prev_time = ros::Time::now().toSec();

    FoE_msg.data.resize(2);
    int32_t pub_x;
    pub_x = (int64_t)(FoE_x);
    int32_t pub_y;
    pub_y = (int64_t)(FoE_y);
    std::vector<int64_t> msgArray = {pub_x, pub_y};

    OF_rec_file << ros::Time::now().toNSec() << "," << pub_x << "," << pub_y << std::endl;

    FoE_msg.data = msgArray;
    // Publish the FOE onto its topic
    FoE_pub.publish(FoE_msg);
  }
}

void opticflowCallback(const dvs_of_msg::FlowPacketMsgArray::ConstPtr &msg) // gets the optic flow from the
{

  for (int i = 0; i < msg->flowpacketmsgs.size(); i = i + 1)
  {
    FlowPacket OFvec;
    OFvec.t = (uint64_t)(msg->flowpacketmsgs[i].t);

    // if (OFvec.t > 1e5) {
    OFvec.x = (int16_t)(msg->flowpacketmsgs[i].x);
    OFvec.y = (int16_t)(msg->flowpacketmsgs[i].y);
    OFvec.u = (float)(msg->flowpacketmsgs[i].u);
    OFvec.v = (float)(msg->flowpacketmsgs[i].v);
    myOF.push_back(OFvec);
    //}
  }

  // Run FOE estimation at rate_
  if (ros::Time::now().toSec() - prev_time >= period_)
  {

    // Fill OF buffers and run FOE estimation

    // double beforetime = ros::Time::now().toSec();

    // Run FOE estimation
    estimationServer();
    // Add 0.5 to FoE_x for truncating by compiler to integer
    // double calctime = ros::Time::now().toSec() - beforetime;

    // ROS_INFO("estimationServer time: %f", calctime);
  }
}

void log_OF(std::vector<FlowPacket> *myOF)
{
  // Fill final_buffer with current OF vector and clear myOF for new OF from subscription.
  FlowPacket OFvec_buf;

  prepMutex.lock();
  for (std::vector<FlowPacket>::iterator it = myOF->begin(); it != myOF->end(); it++)
  {
    uint64_t diff = last_ts + period_;
    uint64_t currenttime = (*it).t;

    if (currenttime >= diff)
    {

      last_ts = (*it).t;
      OFvec_buf.x = (*it).x;
      OFvec_buf.y = (*it).y;
      OFvec_buf.u = (*it).u;
      OFvec_buf.v = (*it).v;
      OFvec_buf.t = (*it).t;
      final_buffer.push_back(OFvec_buf);
    }
  }
  prepMutex.unlock();
  myOF->clear();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "foe_estimator");
  ros::NodeHandle n;

  // Initialize subscribers and publishers
  ros::Subscriber sub = n.subscribe("/OpticFlow", 1, opticflowCallback);
  FoE_pub = n.advertise<std_msgs::Int64MultiArray>("/FoE", 1);

  std::string myDate = currentDateTime();

  std::string filename = "FoE_recording_" + myDate + ".txt";
  OF_rec_file.open(filename);

  ros::spin();

  return 0;
}
