#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

class DataReader {
public:
    static std::vector<std::vector<double>> readDataFile(const std::string& filename, char sep) {
        std::vector<std::vector<double>> data;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return data;
        }
        
        std::string line;
        int lineNumber = 0;
        
        while (std::getline(file, line)) {
            lineNumber++;
            
            // Skip empty lines
            if (line.empty()) {
                continue;
            }
            
            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;
            
            // Parse comma-separated values
            while (std::getline(ss, cell, sep)) {
                try {
                    // Remove any whitespace
                    cell.erase(0, cell.find_first_not_of(" \t"));
                    cell.erase(cell.find_last_not_of(" \t") + 1);
                    
                    if (!cell.empty()) {
                        double value = std::stod(cell);
                        row.push_back(value);
                    }
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Warning: Invalid number '" << cell 
                              << "' on line " << lineNumber << std::endl;
                } catch (const std::out_of_range& e) {
                    std::cerr << "Warning: Number out of range '" << cell 
                              << "' on line " << lineNumber << std::endl;
                }
            }
            
            if (row.size() == 6 || row.size() == 3) {
                data.push_back(row);
            } else {
                std::cerr << "Warning: Line " << lineNumber 
                          << " has " << row.size() << " values, expected 6" << std::endl;
            }
        }
        
        file.close();
        return data;
    }
    
    // Helper function to print the data (useful for debugging)
    static void printData(const std::vector<std::vector<double>>& data, int maxRows = 10) {
        std::cout << "Data format: [x_disp, y_disp, theta_disp, x_vel, y_vel, theta_vel]" << std::endl;
        std::cout << "Showing first " << std::min(maxRows, (int)data.size()) << " rows:" << std::endl;
        
        for (int i = 0; i < std::min(maxRows, (int)data.size()); i++) {
            std::cout << "Row " << i << ": ";
            for (double val : data[i]) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
        
        if (data.size() > maxRows) {
            std::cout << "... and " << (data.size() - maxRows) << " more rows" << std::endl;
        }
    }
};


class DesmosExporter {
public:
    // Export X-Y trajectory
    static void exportXYTrajectory(const std::vector<std::vector<double>>& data, 
                                   const std::string& filename = "xy_trajectory.txt") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not create " << filename << std::endl;
            return;
        }
        
        file << std::fixed << std::setprecision(4);
        
        // Calculate cumulative position
        double x_pos = 0, y_pos = 0;
        
        for (const auto& row : data) {
            file << "(" << row.at(0) << "," << row.at(1) << "), ";
        }
        
        file.close();
    }
    
    // Export time-based data (time vs any parameter)
    static void exportTimeSeriesData(const std::vector<std::vector<double>>& data, 
                                     int parameter_index, 
                                     const std::string& parameter_name,
                                     double dt = 0.1) {  // time step
        std::string filename = parameter_name + "_vs_time.txt";
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not create " << filename << std::endl;
            return;
        }
        
        file << std::fixed << std::setprecision(4);
        
        for (size_t i = 0; i < data.size(); i++) {
            double time = i * dt;
            file << "(" << time << "," << data[i][parameter_index] << ")" << std::endl;
        }
        
        file.close();
        std::cout << parameter_name << " vs time exported to " << filename << std::endl;
    }
    
    // Export velocity magnitude over time
    static void exportVelocityMagnitude(const std::vector<std::vector<double>>& data,
                                        double dt = 0.1) {
        std::ofstream file("velocity_magnitude.txt");
        if (!file.is_open()) {
            std::cerr << "Error: Could not create velocity_magnitude.txt" << std::endl;
            return;
        }
        
        file << std::fixed << std::setprecision(4);
        
        for (size_t i = 0; i < data.size(); i++) {
            double time = i * dt;
            double vx = data[i][3];  // x velocity
            double vy = data[i][4];  // y velocity
            double v_mag = sqrt(vx*vx + vy*vy);
            file << "(" << time << "," << v_mag << ")" << std::endl;
        }
        
        file.close();
        std::cout << "Velocity magnitude exported to velocity_magnitude.txt" << std::endl;
    }
    
    // Export all data as separate lists for Desmos
    static void exportAllForDesmos(const std::vector<std::vector<double>>& data,
                                   double dt = 0.1) {
        std::cout << "\n=== DESMOS READY DATA ===\n" << std::endl;
        
        // Time list
        std::cout << "Time values (copy this into Desmos as t):" << std::endl;
        std::cout << "t = [";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) std::cout << ",";
            std::cout << std::fixed << std::setprecision(2) << i * dt;
        }
        std::cout << "]" << std::endl << std::endl;
        
        // X positions (cumulative)
        std::cout << "X positions (copy this into Desmos as x_pos):" << std::endl;
        std::cout << "x_{pos} = [";
        double x_cumulative = 0;
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) std::cout << ",";
            x_cumulative += data[i][0];
            std::cout << std::fixed << std::setprecision(4) << x_cumulative;
        }
        std::cout << "]" << std::endl << std::endl;
        
        // Y positions (cumulative)
        std::cout << "Y positions (copy this into Desmos as y_pos):" << std::endl;
        std::cout << "y_{pos} = [";
        double y_cumulative = 0;
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) std::cout << ",";
            y_cumulative += data[i][1];
            std::cout << std::fixed << std::setprecision(4) << y_cumulative;
        }
        std::cout << "]" << std::endl << std::endl;
        
        // X velocities
        std::cout << "X velocities (copy this into Desmos as v_x):" << std::endl;
        std::cout << "v_x = [";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) std::cout << ",";
            std::cout << std::fixed << std::setprecision(4) << data[i][3];
        }
        std::cout << "]" << std::endl << std::endl;
        
        // Y velocities
        std::cout << "Y velocities (copy this into Desmos as v_y):" << std::endl;
        std::cout << "v_y = [";
        for (size_t i = 0; i < data.size(); i++) {
            if (i > 0) std::cout << ",";
            std::cout << std::fixed << std::setprecision(4) << data[i][4];
        }
        std::cout << "]" << std::endl << std::endl;
        
        std::cout << "=== HOW TO USE IN DESMOS ===" << std::endl;
        std::cout << "1. Copy each list above into Desmos" << std::endl;
        std::cout << "2. To plot X-Y trajectory: (x_{pos}, y_{pos})" << std::endl;
        std::cout << "3. To plot velocity vs time: (t, v_x) or (t, v_y)" << std::endl;
        std::cout << "4. To plot position vs time: (t, x_{pos}) or (t, y_{pos})" << std::endl;
        std::cout << "=========================" << std::endl;
    }
};
