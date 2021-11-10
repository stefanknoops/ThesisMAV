

#include "EstimateClusterTTC.h"
#include <cmath>
#include <math.h>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <numeric>

// Function Definitions
void EstimateClusterTTC(const std::vector<double> &x_norm, const std::vector<double> &y_norm, const std::vector<double> &TTC,std::vector<std::vector<double>> &ClusterBoxes, std::vector<std::vector<uint>> &clusters)
{
    for (int i = 0; i < clusters.size(); i++)
    {
        int Index = clusters[i][0];
        double LeftBound = x_norm[Index];
        double BottomBound = y_norm[Index];
        double RightBound = x_norm[Index];
        double UpperBound = y_norm[Index];
        double TTCSum;
        // Box = {left bound, bottom bound, right bound, upper bound, TTC}
        for (int j = 0; j < clusters[j].size(); j++)
        {
            Index = clusters[i][j];
            if (x_norm[Index] < LeftBound)
            {
                LeftBound = x_norm[Index];
            }
            else if (x_norm[Index] > RightBound)
            {
                RightBound = x_norm[Index];
            }
            if (y_norm[Index] < BottomBound)
            {
                BottomBound = y_norm[Index];
            }
            else if (y_norm[Index] > UpperBound)
            {
                UpperBound = y_norm[Index];
            }
            TTCSum += TTC[Index];
        };
        ClusterBoxes.push_back({LeftBound,BottomBound,RightBound,UpperBound,TTCSum/clusters[i].size() });
    };



    
};
