/**
 * @file main.cpp for occupancy grid
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-12-17
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <iostream>
#include <math.h>
#include <vector>
#include <matplot/matplot.h>


using namespace std;

/**
 * Sensor characteristic: Min and Max ranges of the beams
 */
double Zmax = 5000, Zmin = 170;

/**
 * Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
 */
double l0 = 0, locc = 0.4, lfree = -0.4;

/**
 * Grid dimensions
 */
double gridWidth = 100, gridHeight = 100;

/**
 * Map dimensions
 */
double mapWidth = 30000, mapHeight = 15000;

/**
 * Robot size with respect to the map
 */
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;

/**
 * Defining an l vector to store the log odds values of each cell
 */
vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[]){
    /**
     * Defining Sensor Characteristics
     */
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;

    //******************Compute r and phi**********************//
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************Evaluate the three cases**********************//
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        return l0;
    }else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        return locc;
    }else if (r <= Zk) {
        return lfree;
    }else{
        cerr << "Something is wrong! cell unknown!" << endl;
    }
    return 0.0;
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    //******************Code the Occupancy Grid Mapping Algorithm**********************//
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
            double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
            if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax) {
                l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            }
        }
    }
}

void visualization()
{
    //Graph Format
    matplot::title("Map");
    matplot::xlim({0, (mapWidth / gridWidth)});
    matplot::ylim({0, (mapHeight / gridHeight)});
    vector<double> x_u;
    vector<double> y_u;
    vector<double> x_f;
    vector<double> y_f;
    vector<double> x_o;
    vector<double> y_o;

    // Draw every grid of the map:
    for (double x = 0; x < mapWidth / gridWidth; x++) {
        cout << "Remaining Rows= " << mapWidth / gridWidth - x << endl;
        for (double y = 0; y < mapHeight / gridHeight; y++) {
            if (abs(l[x][y]) < 0.01) {
                //Unkown state
                x_u.push_back(x);
                y_u.push_back(y);
            }
            else if (l[x][y] >= 0.01) {
                //occupied state
                x_o.push_back(x);
                y_o.push_back(y);
            }
            else if (l[x][y] <= -0.01) {
                //free state
                x_f.push_back(x);
                y_f.push_back(y);
            }
        }
    }

    matplot::hold(matplot::on);
    matplot::plot(x_u, y_u, "g.");

    matplot::hold(matplot::on);
    matplot::plot(x_o, y_o, "k.");

    matplot::hold(matplot::on);
    matplot::plot(x_f, y_f, "r.");

    matplot::show();
    matplot::save("./Images/Map.png");
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("/home/atta/occupancy_grid_mapping/src/Data/poses.txt", "r");
    FILE* measurementFile = fopen("/home/atta/occupancy_grid_mapping/src/Data/measurement.txt", "r");
    cout << "Files are read" << endl;

    /**
     * Scanning the files and retrieving measurement and poses at each timestamp
     */
    int counter{0};
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }

        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
        counter += 1;
    }

    // Visualize the map at the final step
    cout << "Wait for the image to generate" << endl;
    visualization();
    cout << "Done!" << endl;

    return 0;
}

