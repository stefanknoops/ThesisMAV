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

// Include files
#include "estimateFoECPP.h"
#include "cpp_foe/cpp_foe.h"
#include "signum.h"
#include <cmath>
#include <cstring>
#include <random>

// Function Definitions
void estimateFoECPP(std::vector<FlowPacket> OpticFlow, double *FoE_x,
                    double *FoE_y)
{

  int ArraySize = OpticFlow.size();
  int i;
  std::vector<std::vector<double>> norm_lines;
  std::vector<std::vector<double>> all_lines;
  std::vector<std::vector<double>> all_rules;

  std::vector<std::vector<double>> InitialHullLines = {{0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {0, -180, -240}};
  std::vector<std::vector<double>>InitialHullRules = {{0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {0, -180, -240}};
  std::vector<std::vector<double>>InitialHullPoints = {{0,0},{0,240},{180,240},{180,0}};
  std::vector<std::vector<double>>BestHull;
  std::vector<std::vector<double>> HullLines;
  std::vector<std::vector<double>> HullRules;
  std::vector<std::vector<double>> HullPoints;
  std::vector<double> RandomVectorRule;
  double MaxScore = 0;
  bool Illegal;



  int AmountOfIterations = 50;


  double FOEX = 0;
  double FOEY = 0;
  
  std::random_device RandomSeed;
  std::mt19937 RandomNumber(RandomSeed());
  std::uniform_int_distribution<> RandomNumberDistribution(0, ArraySize);

 

  //  Set method parameters
  //  Set # iterations for RANSAC search
  //  Set # failures allowed in iteration before stopping search
  //  Initialize frame boundaries to hull
  //  Rules: [A, B, C, sign(v), sign(u)]
  //  Initialize optical flow variables
  //  Write into ax + by + c = 0 ==> perp equation = bx-ay + lambda= 0

  ArraySize = OpticFlow.size(); //set loop length to the length of the vector containing optic flow
  for (i = 0; i < ArraySize; i++)
  {
    norm_lines[i][0] = OpticFlow[i].v;
    norm_lines[i][1] = -OpticFlow[i].u;
    norm_lines[i][2] = OpticFlow[i].x * (OpticFlow[i].y + OpticFlow[i].v) - (OpticFlow[i].x + OpticFlow[i].u) * OpticFlow[i].y;
    std::cout << "Arraysize line 86:" << ArraySize;
  }

  //  Create a list with the rules for all vectors
  //  Rule: [A, B, C, sign(v), sign(u)] // not sure what rules are
  //  Put line descriptions in same format for halfplane check (change sign if negative)

  for (i = 0; i < ArraySize; i++)
  {
    double sign_v = (OpticFlow[i].v > 0) - (OpticFlow[i].v > 0); //source: google cpp signum
    double sign_u = (OpticFlow[i].u > 0) - (OpticFlow[i].u > 0); //source: google cpp signum
    if (norm_lines[i][0] < 0)
    {
      all_rules[i] = std::vector<double>{-norm_lines[i][0], -norm_lines[i][1], -norm_lines[i][2], sign_v, sign_u};
    }
    else
    {
      all_rules[i] = std::vector<double>{norm_lines[i][0], norm_lines[i][1], norm_lines[i][2], sign_v, sign_u};
    }
  }

  //add initial hull to all lines

  all_lines = norm_lines;
  for (i = 0; i < InitialHullLines.size(); i++)
  {
    std::vector<double> append_vector = {InitialHullLines[i]};
    all_lines.push_back(append_vector);
  }

  for (int n = 0; n < AmountOfIterations; n++)
  { //for n iterations
    std::cout << "Begin for loop"
    //initiate hull
    std::vector<bool> VectorInHull(norm_lines.size(), false);
    std::vector<bool> VectorTried(norm_lines.size(), false);

    HullLines = InitialHullLines;
    HullRules = InitialHullRules;
    HullPoints = InitialHullPoints;

    bool StopSearch = false;
    int StopCount = 0;
    int MaxFailures = 1;

    //set all vectors to not tried

    while (!StopSearch) //while not stopping search
    {                   //take random vector and check whether it has not been used yet
      double RandomVectorIndex = RandomNumberDistribution(RandomNumber);
      while (VectorTried[RandomVectorIndex] == 1 || VectorInHull[RandomVectorIndex] == 1)
      {
        double RandomVectorIndex = RandomNumberDistribution(RandomNumber);
      }

      VectorTried[RandomVectorIndex] = true;

      //see if all vectors are tried yet
      if (std::all_of(VectorTried.begin(), VectorTried.end(), [](bool v)
                      { return v; }))
      {
        StopSearch == true;
      }

      //choose vector
      std::vector<double> RandomVector = norm_lines[RandomVectorIndex];
      double sign_v = signum(OpticFlow[i].v); //source: google cpp signum
      double sign_u = signum(OpticFlow[i].u); //source: google cpp signum

      //put vector in standard format, add index
      if (RandomVector[0] < 0)
      {
        std::vector<double> RandomVectorRule = {-RandomVector[0], -RandomVector[1], -RandomVector[2], sign_v, sign_u, RandomVectorIndex};
      }
      else
      {
        std::vector<double> RandomVectorRule = {RandomVector[0], RandomVector[1], RandomVector[2], sign_v, sign_u, RandomVectorIndex};
      }

      bool AddCheck = false;
      int IllegalCount = 0;

      for (int i = 0; i < HullLines.size(); i++) //for all hull lines
      {
        //if not parallel
        std::vector<double> MergeMatrixA = {HullLines[i][0], HullLines[i][1], RandomVector[0], RandomVector[1]};
        std::vector<double> MergeMatrixC = {-HullLines[i][2], -RandomVector[2]};

        double DeterminantA = MergeMatrixA[0] * MergeMatrixA[3] - MergeMatrixA[1] * MergeMatrixA[2];
        if (DeterminantA > 0)
        {
          //calculate intersection
          //x = (c1b2 - c2b1)/determinant
          //y = (a1c2 - a2c1)/determinant

          double IntersectionX = (HullLines[i][2] * RandomVector[1] - HullLines[i][1] * RandomVector[2]) / DeterminantA;
          double IntersectionY = (HullLines[i][0] * RandomVector[2] - HullLines[i][2] * RandomVector[1]) / DeterminantA;

          Illegal = false;

          for (int j = 0; j < HullRules.size(); j++) //for all rules
          {
            /* code */
            //check whether vector does not violate rules
            std::vector<double> CurrentRule = HullRules[j];
                    if (signum(CurrentRule[0]) == 1  && (signum(CurrentRule[2]) == 1) || signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == -1) ||signum(CurrentRule[1]) == 0  && (signum(CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (signum(CurrentRule[3])== 1))) { //AADDD SIGN FUNCTION HERE
              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];
              if (IntersectionCalculation != 0)
              {
                Illegal = true;
                IllegalCount += 1;
              }

                    } else if (signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == 1) || signum(CurrentRule[1]) == 1 && (signum(CurrentRule[3]) == -1) || signum(CurrentRule[1]) == 0  && (signum(CurrentRule[4]) == -1) || signum(CurrentRule[0]) && (signum(CurrentRule[3]) == -1)) { //AADDD SIGN FUNCTION HERE
              double IntersectionCalculation = CurrentRule[0] * IntersectionX + CurrentRule[1] * IntersectionY + CurrentRule[2];
              if (IntersectionCalculation != 0)
              {
                Illegal = true;
                IllegalCount += 1;
              }
                    }
                

                

                  //if not violated, add to hull
                  if (!Illegal && !AddCheck) {
              HullLines.push_back(RandomVector);
              HullRules.push_back(RandomVectorRule);
              VectorInHull[RandomVectorIndex] = true;
              AddCheck = true;
                  }

                  if (!Illegal) {
              HullPoints.push_back({IntersectionX, IntersectionY});
              AddCheck = true;
                  }

                  if (IllegalCount == HullRules.size()) {
              StopCount += 1;
              if (StopCount == MaxFailures)
              {
                StopSearch = true;
              }
                  }

                    //if not contributing, dismiss vector
          }
        } else
        {
          IllegalCount += 1;
        }
      }

      std::vector<int> PointsToRemove;
      for (int i = 0; i < HullPoints.size(); i++) //for all hull points
      {
        /* code */
        //check latest rule
        double CurrentPointX = HullPoints[i][0];
        double CurrentPointY = HullPoints[i][1];

        std::vector<double> CurrentRule = HullRules.back();

        if (signum(CurrentRule[0]) == 1  && (signum(CurrentRule[2]) == 1) || signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == -1) ||signum(CurrentRule[1]) == 0  && (signum(CurrentRule[4]) == 1) || (signum(CurrentRule[0]) == 0 && (signum(CurrentRule[3])== 1))) { //AADDD SIGN FUNCTION HERE  //AADDD SIGN FUNCTION HERE
          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation != 0)
          {
            PointsToRemove.push_back(i);
            Illegal = true;
          }
        } else if ((signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == 1) || signum(CurrentRule[1]) == 1 && (signum(CurrentRule[3]) == -1) || signum(CurrentRule[1]) == 0  && (signum(CurrentRule[4]) == -1) || signum(CurrentRule[0]) && (signum(CurrentRule[3]) == -1))) { //AADDD SIGN FUNCTION HERE
          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation != 0)
          {
            Illegal = true;
            PointsToRemove.push_back(i);
          }
          }
      }

      for (int i = PointsToRemove.size() - 1; i >= 0; i--) //from back to front to avoid changing the index halfway
      {
        HullPoints.erase(HullLines.begin() + PointsToRemove[i]);
      }

      std::vector<int> LinesToRemove;
      for (int i = 0; i < HullLines.size(); i++) //for all alines
      {
        bool NonZeroCheck = false;
        std::vector<double> CurrentRule = HullRules[i];
        for (int j = 0; j < HullPoints.size(); j++)
        {
          double CurrentPointX = HullPoints[i][0];
          double CurrentPointY = HullPoints[i][1];
          double IntersectionCalculation = CurrentRule[0] * CurrentPointX + CurrentRule[1] * CurrentPointY + CurrentRule[2];
          if (IntersectionCalculation != 0)
          {
            NonZeroCheck = true;
          }
        }
        if (NonZeroCheck)
        {
          LinesToRemove.push_back(i);
          VectorInHull[RandomVectorIndex] = false;
        }
      }

      for (int i = LinesToRemove.size() - 1; i >= 0; i--) //from back to front to avoid changing the index halfway
      {
        HullLines.erase(HullLines.begin() + LinesToRemove[i]);
        HullRules.erase(HullLines.begin() + LinesToRemove[i]);
      }

      //remove lines and rules
    }

    //calculate score
    double IterationX = 0;
    double IterationY = 0;
    int InlierCount = 0;
    int IterationScore;
    for (int i; i < HullPoints.size(); i++)
    {
      IterationX += HullPoints[i][0];
      IterationY += HullPoints[i][1];
    }
    IterationX = IterationX / HullPoints.size();
    IterationY = IterationY / HullPoints.size();

    for (int i; i < all_rules.size(); i++)
    {
      std::vector<double> CurrentRule = all_rules[i];
      if (signum(CurrentRule[0]) == 1 && (signum(CurrentRule[2]) == 1) || signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == -1) || signum(CurrentRule[1]) == 0 && (signum(CurrentRule[4]) == 1) || signum(CurrentRule[0]) == 0 && (signum(CurrentRule[3]) == 1))
      { //AADDD SIGN FUNCTION HERE
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation != 0)
        {
          InlierCount += 1;
        }
      } else if ((signum(CurrentRule[1]) == -1 && (signum(CurrentRule[3]) == 1) || signum(CurrentRule[1]) == 1 && (signum(CurrentRule[3]) == -1) || signum(CurrentRule[1]) == 0  && (signum(CurrentRule[4]) == -1) || signum(CurrentRule[0]) && (signum(CurrentRule[3]) == -1))) { //AADDD SIGN FUNCTION HERE
        double IntersectionCalculation = CurrentRule[0] * IterationX + CurrentRule[1] * IterationY + CurrentRule[2];
        if (IntersectionCalculation != 0)
        {
          InlierCount += 1;
        }
      }
    }

    IterationScore = InlierCount;

    if (IterationScore > MaxScore)
    {
      MaxScore = IterationScore;
      BestHull = HullPoints;
    }
  }


  for (int i; i < HullPoints.size(); i++)
  {
    FOEX += HullPoints[i][0];
    FOEY += HullPoints[i][1];
  }
  FOEX = FOEX / HullPoints.size();
  FOEY = FOEY / HullPoints.size();
}

