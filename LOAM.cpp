//#include <list>
#include <vector>
#include <cmath>
#include <ctime>
#include <cfloat>
#include <array>

class LEGO_cloudhandler{
    public:
        float rangematrix[16][1800];
        int index[28800];
        int groundmetrix[28800];
        //std::list<int> groundpoint;
        std::vector<int> groundpoint;
        int labelmatrix[16][1800];
        void pointcloudproject(float array[28800][5]);
        void markground(float array[28800][5]);
        void labelcomponents(int row, int col);

};
void LEGO_cloudhandler::pointcloudproject(float array[28800][5]){
    for (int i=0; i<28800; i++){
        float x = array[i][0];
        float y = array[i][1];
        float z = array[i][2];
        int rowID = array[i][3];
        int colID = array[i][4];
        float distance = sqrt(x*x+y*y+z*z);
        if (distance<1.0){
            continue;
        }
        rangematrix[rowID][colID]=distance;
        int indexnum = colID + rowID*1800;
        if (i>=28800){
            continue;
        }
        index[i]=indexnum;
    }
}
void LEGO_cloudhandler::markground(float array[28800][5]){
    for (int i = 0; i < 1800; i++){
        for (int j =0; j < 4; j++){
            int lowerID = i * 16 + j;
            int upperID = i * 16 + (j+1);
            float x1 = array[lowerID][0];
            float y1 = array[lowerID][1];
            float z1 = array[lowerID][2];
            float x2 = array[upperID][0];
            float y2 = array[upperID][1];
            float z2 = array[upperID][2];
            float disx = x1 - x2;
            float disy = y1 - y2;
            float disz = z1 - z2;
            float angle = atan2(disz, sqrt(disx*disx+disy*disy)) * 180 / M_PI;
            if (abs(angle)<=10.0){
                groundmetrix[upperID]=1;
                groundmetrix[lowerID]=1;
                groundpoint.push_back(lowerID);
            }
        }
    }
} void LEGO_cloudhandler::labelcomponents(int row, int col){
    int labelcount = 1;
    int queueIndX[28800];
    int queueIndY[28800];
    int allPushedIndX[28800];
    int allPushedIndY[28800];
    float list_compare[2] = {float(0), float(0)};
    queueIndX[0] = row;
    queueIndY[0] = col;
    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    float angle_hor = 0.2/180 * M_PI;
    float angle_ver = 2/180 * M_PI;
    float angle_bon = 60/180 * M_PI;
    int queuesize = 1;
    int queuestartInd = 0;
    int queueendInd = 1;
    int allpushedindsize = 1;
    while (queuesize > 0){
        int findpointIDX = queueIndX[queuestartInd];
        int findpointIDY = queueIndY[queueendInd];
        queuesize = queuesize - 1;
        queuestartInd = queuestartInd +1;
        labelmatrix[findpointIDX][findpointIDY] = labelcount;
        if (findpointIDX + 1 < 0 || findpointIDX - 1 < 0 || findpointIDX + 1 > 16 || findpointIDX - 1 > 16){
            continue;
        }
    }
}
