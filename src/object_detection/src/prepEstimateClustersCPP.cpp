/**
 * This file is part of the object_detection package - MAVLab TU Delft
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
#include "prepEstimateClustersCPP.h"

// Function Declarations

void prepEstimateClustersCPP(std::vector<FlowPacket> &optic_flow, double FoE_x_estimate, std::vector<double> &OpticFlowXnorm, std::vector<double> &OpticFlowYnorm,
                             std::vector<double> &TTC)
{
  int ArraySize = optic_flow.size();

  std::vector<double> OpticFlowX;
  std::vector<double> OpticFlowY;
  std::vector<double> OpticFlowMagnitude;

  std::vector<double> OpticFlowu;
  std::vector<double> OpticFlowv;

  // init magnitude
  double AverageMagnitude = 0;
  double StandardDevMagnitude;

  // define pi
  double pi = atan(1) / 4;

  // calc average magnitude and std magnitude

  for (int i = 0; i < ArraySize; i++)
  {

    double Magnitude = sqrt(pow(optic_flow[i].u, 2) + pow(optic_flow[i].v, 2));

    OpticFlowMagnitude.push_back(Magnitude);
    AverageMagnitude += Magnitude;
  }
  AverageMagnitude = AverageMagnitude / ArraySize;

  double SquaredSumMagnitude = std::inner_product(OpticFlowMagnitude.begin(), OpticFlowMagnitude.end(), OpticFlowMagnitude.begin(), 0.0);
  StandardDevMagnitude = std::sqrt(SquaredSumMagnitude / OpticFlowMagnitude.size() - AverageMagnitude * AverageMagnitude); // this may break, recheck later

  // set threshold for rejection
  double MagnitudeThreshold = AverageMagnitude + StandardDevMagnitude * 2;

  // only save OF vectors that meet threshold value
  for (int i = 0; i < ArraySize; i++)
  {

    if (OpticFlowMagnitude[i] > MagnitudeThreshold)
    {

      OpticFlowX.push_back(optic_flow[i].x);
      OpticFlowY.push_back(optic_flow[i].y);
      OpticFlowu.push_back(optic_flow[i].u);
      OpticFlowv.push_back(optic_flow[i].v);
      OpticFlowXnorm.push_back(optic_flow[i].x / 240);
      OpticFlowYnorm.push_back(optic_flow[i].y / 180);
    }
    else // this might be skipped?
    {
      OpticFlowMagnitude.erase(OpticFlowMagnitude.begin() + i);
      i--;
    }
  }

  // calc average
  double AverageFlowu = std::accumulate(OpticFlowu.begin(), OpticFlowu.end(), 0.0) / OpticFlowu.size();
  double SquaredSumFlowu = std::inner_product(OpticFlowu.begin(), OpticFlowu.end(), OpticFlowu.begin(), 0.0);
  double StandardDevFlowu = std::sqrt(SquaredSumFlowu / OpticFlowu.size() - AverageFlowu * AverageFlowu); // this may break, recheck later

  int NewArraySize = OpticFlowMagnitude.size();
  // std::vector<double> TTC;

  for (int i = 0; i < NewArraySize; i++)
  {
    double NormalizedU = (OpticFlowu[i] - AverageFlowu) / StandardDevFlowu;
    double CalcTTC = std::abs((OpticFlowX[i] - FoE_x_estimate) / NormalizedU);
    if (CalcTTC > 3000)
    {
      TTC.push_back(CalcTTC);
    }
  }
  double AverageTTC = std::accumulate(TTC.begin(), TTC.end(), 0.0) / TTC.size();
  double SquaredSumTTC = std::inner_product(TTC.begin(), TTC.end(), TTC.begin(), 0.0);
  double StandardDevTTC = std::sqrt(SquaredSumTTC / TTC.size() - AverageTTC * AverageTTC); // this may break, recheck later

  // Normalize TTC
  int TTCArraySize = TTC.size();
  for (int i = 0; i < TTCArraySize; i++)
  {
    TTC[i] = (TTC[i] - AverageTTC) / StandardDevTTC;
  };
}
