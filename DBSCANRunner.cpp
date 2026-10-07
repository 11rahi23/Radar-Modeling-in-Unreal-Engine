#include "DBSCANRunner.h"


DBSCANRunner::DBSCANRunner(ARadar* currRadar)
{
    _currentRadar = currRadar;
    _eps = currRadar->epsilon;
    _minPts = currRadar->minimumSamples;
}

DBSCANRunner::~DBSCANRunner()
{
}

bool DBSCANRunner::Init()
{
    bStopThread = false;
    return true;
}

uint32 DBSCANRunner::Run() {
    while (!bStopThread) {
        if (!_currentRadar->lidarScanQueue.IsEmpty()) {
            _currentRadar->lidarScanQueue.Dequeue(scannedPoints);
            _currentRadar->lidarScanTimeQueue.Dequeue(scanTime);
            _numPts = scannedPoints.Num();
            int rows = scannedPoints.Num();
            int cols = 3;
            double** data = new double* [rows];
            int* labels = new int[rows];
            bool* coreSamples = new bool[rows];
            
            for (int i = 0; i < rows; i++) {
                data[i] = new double[cols];
            }
            for (int i = 0; i < rows; i++) {
                data[i][0] = scannedPoints[i].position.X;
                data[i][1] = scannedPoints[i].position.Y;
                data[i][2] = scannedPoints[i].position.Z;
            }
            int* noise = new int[rows];
            DBSCAN<3>(rows, *data, _eps, _minPts, coreSamples, noise, labels);
        }
    }
    return 0;
}

void DBSCANRunner::Stop()
{
    bStopThread = true;
}
