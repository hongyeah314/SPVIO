#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>


using namespace std;

struct IMUData {
    double d_time;
    double x, y, z;
};
struct biasdata {
    double d_time;
    double x, y, z;
};
struct yprdata {
    double d_time;
    double yaw, pitch, roll;
};
struct ceresdata {
    double d_time;
    double cost;
};

std::vector<IMUData> extractIMUData(const std::string& filename) {
    std::vector<IMUData> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return data;
    }

    std::string line;
    std::string to_remove = "un_acc_0:";

    while (std::getline(file, line)) {
        if (line.find("un_acc_0:") != std::string::npos) {

            // size_t position = line.find(to_remove);
            // line.erase(position, to_remove.length());
            std::istringstream iss(line);
            std::string time,un_acc;
            IMUData entry;
            iss >> time;
            iss >> entry.d_time;
            iss >> un_acc;
            iss >> entry.x >> entry.y >> entry.z;
            data.push_back(entry);
        }
    }

    return data;
}

std::vector<biasdata> extractbiasData(const std::string& filename) {
    std::vector<biasdata> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return data;
    }

    std::string line;


    while (std::getline(file, line)) {
        if (line.find("bais dector:") != std::string::npos) {

            // size_t position = line.find(to_remove);
            // line.erase(position, to_remove.length());
            std::cout<<line<<std::endl;
            std::istringstream iss(line);
            std::string time,bais;
            biasdata entry;
            iss >> bais;
            iss >> time;
            iss >> entry.d_time;
            iss >> entry.x >> entry.y >> entry.z;
            data.push_back(entry);
        }
    }

    return data;
}


std::vector<yprdata> extractyprData(const std::string& filename) {
    std::vector<yprdata> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return data;
    }

    std::string line;


    while (std::getline(file, line)) {
        if (line.find("R dector:") != std::string::npos) {

            // size_t position = line.find(to_remove);
            // line.erase(position, to_remove.length());
            std::istringstream iss(line);
            std::string time,R;
            yprdata entry;
            iss >> R;
            iss >> time;

            iss >> entry.d_time;
            iss >> entry.yaw >> entry.pitch >> entry.roll;
            data.push_back(entry);
        }
    }

    return data;
}

std::vector<ceresdata> extractceresData(const std::string& filename) {
    std::vector<ceresdata> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return data;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("Initial ") != std::string::npos) {
            // size_t position = line.find(to_remove);
            // line.erase(position, to_remove.length());
            std::istringstream iss(line);
            std::string time,ceres;
            ceresdata entry;
            iss >> ceres;
            iss >> entry.cost;
            data.push_back(entry);
        }
    }
    return data;
}

void plotIMUData(const std::vector<IMUData>& data) {
    std::cout << "plotting IMU data" << std::endl;
    //检查当前目录下是否有data.txt文件
    if (std::ifstream("./data.txt")) {
        std::cout << "data.txt already exists" << std::endl;
        //删除并新建一个data.txt文件
        std::remove("./data.txt");
    }
    std::ofstream outfile("./data.txt");

    for (const auto& entry : data) {
        //std::cout << std::fixed << std::setprecision(5) << entry.d_time << " " << entry.x << " " << entry.y << " " << entry.z << std::endl;
        outfile << std::fixed << std::setprecision(10) <<entry.d_time <<" "<<entry.x << " " << entry.y << " " << entry.z << std::endl;
    }
    outfile.close();

    system("/Users/zhanglei/miniconda3/bin/python /Users/zhanglei/code/SPVIO/python/plot.py");
}
void plotyprData(const std::vector<yprdata>& data) {
    std::cout << "plotting IMU data" << std::endl;
    //检查当前目录下是否有data.txt文件
    if (std::ifstream("./data.txt")) {
        std::cout << "data.txt already exists" << std::endl;
        //删除并新建一个data.txt文件
        std::remove("./data.txt");
    }
    std::ofstream outfile("./data.txt");

    for (const auto& entry : data) {
        std::cout << std::fixed << std::setprecision(5) << entry.d_time << " " << entry.yaw << " " << entry.pitch << " " << entry.roll << std::endl;
        outfile << std::fixed << std::setprecision(10) <<entry.d_time <<" "<<entry.yaw << " " << entry.pitch << " " << entry.roll << std::endl;
    }
    outfile.close();

    system("/Users/zhanglei/miniconda3/bin/python /Users/zhanglei/code/SPVIO/python/plot.py");
}
void plotbiasData(const std::vector<biasdata>& data) {
    std::cout << "plotting bias data" << std::endl;
    //检查当前目录下是否有data.txt文件
    if (std::ifstream("./data.txt")) {
        std::cout << "data.txt already exists" << std::endl;
        //删除并新建一个data.txt文件
        std::remove("./data.txt");
    }
    std::ofstream outfile("./data.txt");

    for (const auto& entry : data) {
        std::cout << std::fixed << std::setprecision(5) << entry.d_time << " " << entry.x << " " << entry.y << " " << entry.z << std::endl;
        outfile << std::fixed << std::setprecision(10) <<entry.d_time <<" "<<entry.x << " " << entry.y << " " << entry.z << std::endl;
    }
    outfile.close();

    system("/Users/zhanglei/miniconda3/bin/python /Users/zhanglei/code/SPVIO/python/plot.py");
}

void plotceresData(const std::vector<ceresdata>& data) {
    std::cout << "plotting ceres data" << std::endl;
    //检查当前目录下是否有data.txt文件
    if (std::ifstream("./data.txt")) {
        std::cout << "data.txt already exists" << std::endl;
        //删除并新建一个data.txt文件
        std::remove("./data.txt");
    }
    std::ofstream outfile("./data.txt");
    for (const auto& entry : data) {
        std::cout << std::fixed << std::setprecision(5) << entry.d_time << " " << entry.cost << std::endl;
        outfile << std::fixed << std::setprecision(10) <<entry.d_time <<" "<<entry.cost << std::endl;
    }
    outfile.close();
    system("/Users/zhanglei/miniconda3/bin/python /Users/zhanglei/code/SPVIO/python/plot.py");
}

int main(int argc, char** argv) {
    std::string filename;

    if (argc > 1) {
        filename = argv[1];
    }
    
    auto data = extractIMUData(filename);
    data.erase(data.begin());
    data.pop_back();

    auto biasdata = extractbiasData(filename);
    auto yprdata = extractyprData(filename);
    cout<<"data size: "<<biasdata.size()<<endl;

    auto ceresdata = extractceresData(filename);

    // plotIMUData(data);
    // plotbiasData(biasdata);
    //plotyprData(yprdata);

    cout<<ceresdata.size()<<endl;

    return 0;
}
