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
  int MaxScore = 0;
  int MinScore = 10000;
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

    //std::cout << "OF vector = " << A << "\t" << B << "\t" << C << std::endl;
    //std::cout << "Normal vector = " << B << "\t" << -A << "\t" << D << std::endl;


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

  // all_lines = NormalLines;
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
    std::vector<int> VectorInHull(NormalLines.size(), 0);
    std::vector<int> VectorTried(NormalLines.size(), 0);

    HullLines = InitialHullLines;
    HullRules = InitialHullRules;
    HullPoints = InitialHullPoints;

    bool StopSearch = false;
    int StopCount = 0;
    std::vector<double> RandomNormalRule;

    // set all vectors to not tried

    while (!StopSearch) // while not stopping search
    {                   // take random vector and check whether it has not been used yet

      int RandomNormalIndex = RandomNumberDistribution(RandomNumber);
      if (std::all_of(VectorTried.begin(), VectorTried.end(), [](int x)
                      { return x == 1; }))
      {
        StopSearch = true;
        // ROS_INFO("STOP");
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

      // see if all vectors are tried yet

      // choose vector
      std::vector<double> RandomNormal = NormalLines[RandomNormalIndex];
      //std::cout << "Random vector = " << RandomNormal[0] << "\t" << RandomNormal[1] << "\t" << RandomNormal[2] << std::endl;

      double sign_u = signum(OpticFlow[RandomNormalIndex].u); // source: google cpp signum
      double sign_v = signum(OpticFlow[RandomNormalIndex].v); // source: google cpp signum

      // put vector in standard format, add index
      if (RandomNormal[0] < 0.0)
      {

        RandomNormalRule = {-RandomNormal[0], -RandomNormal[1],-RandomNormal[2], sign_v, sign_u, (double)RandomNormalIndex};
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
            //std::cout << "RV: " << RandomNormal[0] << ", " << RandomNormal[1] << ", " << RandomNormal[2] << std::endl;
            HullLines.push_back(RandomNormal);
            int EndOfVector = HullLines.size()-1;
            //std::cout << "HL" << HullLines[EndOfVector][0] << ",\t " << HullLines[EndOfVector][1] << ",\t " << HullLines[EndOfVector][2] << std::endl;
            HullRules.push_back(RandomNormalRule);
            VectorInHull[RandomNormalIndex] = 1;
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
      BestHullLines = HullLines;
    }
  }

  double FOEX = 0;
  double FOEY = 0;
  for (int i = 0; i < BestHull.size(); i++)
  {
    // ROS_INFO("point %i: %lf, %lf", i, BestHull[i][0], BestHull[i][1]);
    FOEX = FOEX + BestHull[i][0];
    FOEY = FOEY + BestHull[i][1];
    // ROS_INFO("FOEX %lf", FOEX);
    // ROS_INFO("FOEY %lf", FOEY);
  }
  // std::cout << FOEX / (double)BestHull.size() << "," << FOEY / (double)BestHull.size() << std::endl;
  FoE_hist_x.push_back(FOEX / (double)BestHull.size());
  FoE_hist_y.push_back(FOEY / (double)BestHull.size());

  *FoE_x = std::accumulate(FoE_hist_x.begin(), FoE_hist_x.end(), 0.0) / FoE_hist_x.size();
  *FoE_y = std::accumulate(FoE_hist_y.begin(), FoE_hist_y.end(), 0.0) / FoE_hist_y.size();

/*

  tan30 = 120/XX;

  XX = 120/tan30

  FOEangX = atan(FoE_x / (120/tan30)) = atan(FoE_x *tan30 / 120)

  float XX = std::atan(30) * 120;

  float FoE_angle_X = FoE_x/NumPixels_X
*/
  if (FoE_hist_x.size() > 10)
  {
    FoE_hist_x.erase(FoE_hist_x.begin());
    FoE_hist_y.erase(FoE_hist_y.begin());
  }

  int64_t current_time = ros::Time::now().toNSec();

  FoE_rec_file << current_time << "," << FOEX / (double)BestHull.size() << "," << FOEY / (double)BestHull.size() << std::endl;

  for (int i = 0; i < BestHullLines.size(); i++)
  {//std::cout << "besthull" << current_time << "," << BestHullLines[i][0] << "," << BestHullLines[i][1] << "," << BestHullLines[i][2] << std::endl;

    HL_log_file << current_time << "," << BestHullLines[i][0] << "," << BestHullLines[i][1] << "," << BestHullLines[i][2] << std::endl;
  }

  for (int i = 0; i < BestHull.size(); i++)
  {
    HP_log_file << current_time << "," << BestHull[i][0] << "," << BestHull[i][1] << std::endl;
  }

  for (int i = 0; i < ArraySize; i++)
  {
    //std::cout << current_time << "," << 0 << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << std::endl;

    FoE_flow_rec_file << current_time << "," << 0 << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << OpticFlow[i].u << "," << OpticFlow[i].v << std::endl;
    FoE_flow_rec_file << current_time << "," << 1 << "," << OpticFlow[i].x << "," << OpticFlow[i].y << "," << -OpticFlow[i].v << "," << OpticFlow[i].u << std::endl;
  }
}

void estimationServer()
{
  // Fill final OF buffer to be used by FOE estimation
  log_OF(&myOF);
  std::vector<FlowPacket> optic_flow;
  int buffersize = final_buffer.size();
  // ROS_INFO("estimationserver buffersize: %i", buffersize);
  if (final_buffer.size() > 1) // min amount of vectors required
  {

    optic_flow = fillOpticFlowArray();

    // double beforetime = ros::Time::now().toSec();

    // Run FOE estimation
    estimateFoECPP(optic_flow, &FoE_x, &FoE_y);
    // Add 0.5 to FoE_x for truncating by compiler to integer
    // double calctime = ros::Time::now().toSec() - beforetime;

    // ROS_INFO("estimateFoECPP time: %f", calctime);

    prev_time = ros::Time::now().toSec();

    // Publish the FOE onto its topic
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

    FoE_msg.x = (int)FoE_x;
    FoE_msg.y = (int)FoE_y;
    FoE_pub.publish(FoE_msg);

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
  FoE_pub = n.advertise<cpp_foe::FoE>("/FoE", 1);

  std::string myDate = currentDateTime();

  std::string filename = "FoE_recording_" + myDate + ".txt";
  FoE_rec_file.open(filename);

  std::string filename_flow = "FoE_flow_recording_" + myDate + ".txt";

  FoE_flow_rec_file.open(filename_flow);

  std::string filename_HP = "HP_recording" + myDate + ".txt";

  HP_log_file.open(filename_HP);

  std::string filename_HL = "HL_recording" + myDate + ".txt";

  HL_log_file.open(filename_HL);

  ros::spin();

  return 0;
}
