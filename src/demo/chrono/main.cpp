#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <array>
#include "FmuHelper.h"

// Hardcoded paths for demo purposes - in a real app these might be args
// Assuming running from build directory or referencing fixed paths relative to repository root

// Helper for vector/quat names since they are expanded in FMI
struct Vec3Vars { std::string x, y, z; };
struct QuatVars { std::string e0, e1, e2, e3; };

void SetVecVariable(FmuHelper& fmu, const std::string& prefix, const double* v) {
    fmu.SetVariable(prefix + ".x", v[0]);
    fmu.SetVariable(prefix + ".y", v[1]);
    fmu.SetVariable(prefix + ".z", v[2]);
}

void SetQuatVariable(FmuHelper& fmu, const std::string& prefix, const double* q) {
    fmu.SetVariable(prefix + ".e0", q[0]);
    fmu.SetVariable(prefix + ".e1", q[1]);
    fmu.SetVariable(prefix + ".e2", q[2]);
    fmu.SetVariable(prefix + ".e3", q[3]);
}

void GetVecVariable(FmuHelper& fmu, const std::string& prefix, double* v) {
    fmu.GetVariable(prefix + ".x", v[0]);
    fmu.GetVariable(prefix + ".y", v[1]);
    fmu.GetVariable(prefix + ".z", v[2]);
}

void GetQuatVariable(FmuHelper& fmu, const std::string& prefix, double* q) {
    fmu.GetVariable(prefix + ".e0", q[0]);
    fmu.GetVariable(prefix + ".e1", q[1]);
    fmu.GetVariable(prefix + ".e2", q[2]);
    fmu.GetVariable(prefix + ".e3", q[3]);
}


#include "DemoConfiguration.h"

int main(int argc, char* argv[]) {
    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    DemoConfiguration config;
    if (!config.Load("demo_config.json")) {
        std::cerr << "Error: Could not load demo_config.json" << std::endl;
        return 1;
    }

    double step_size = config.GetDouble("simulation.step_size", 2e-3);
    double start_time = config.GetDouble("simulation.start_time", 0.0);
    double t_end = config.GetDouble("simulation.end_time", 15.0);
    
    // FMU Filenames & Paths
    // Helper to get absolute path from config or default
    auto get_abs_path = [&](const std::string& key) {
        std::string p = config.GetString(key, "");
        if(p.empty()) throw std::runtime_error("Missing config: " + key);
        return std::filesystem::absolute(p).string();
    };

    std::string vehicle_fmu_file = get_abs_path("vehicle.fmu_path");
    std::string powertrain_fmu_file = get_abs_path("powertrain.fmu_path");
    std::string driver_fmu_file = get_abs_path("driver.fmu_path");
    std::string tire_fmu_file = get_abs_path("tire.fmu_path");
    std::string terrain_fmu_file = get_abs_path("terrain.fmu_path");

    // Check existence
    if (!std::filesystem::exists(vehicle_fmu_file)) {
        std::cerr << "Error: FMU not found at " << vehicle_fmu_file << std::endl;
        return 1;
    }

    try {
        // ---------------------------------------------------------------------
        // 1. Instantiate FMUs
        // ---------------------------------------------------------------------
        std::cout << "Instantiating FMUs..." << std::endl;
        
        // Ensure directories exist
        auto ensure_dir = [](const std::string& path) {
            if (!std::filesystem::exists(path)) std::filesystem::create_directories(path);
        };

        std::string v_unpack = std::filesystem::absolute(config.GetString("vehicle.unpack_dir", "./tmp")).string();
        std::string p_unpack = std::filesystem::absolute(config.GetString("powertrain.unpack_dir", "./tmp")).string();
        std::string d_unpack = std::filesystem::absolute(config.GetString("driver.unpack_dir", "./tmp")).string();

        ensure_dir(v_unpack);
        ensure_dir(p_unpack);
        ensure_dir(d_unpack);

        FmuHelper vehicle_fmu("WheeledVehicleFMU", vehicle_fmu_file, v_unpack);
        FmuHelper powertrain_fmu("PowertrainFMU", powertrain_fmu_file, p_unpack);
        FmuHelper driver_fmu("DriverFMU", driver_fmu_file, d_unpack);

        std::vector<FmuHelper*> tires;
        std::vector<FmuHelper*> terrains;
        
        std::string t_prefix = config.GetString("tire.unpack_dir_prefix", "./tmp_tire_");
        std::string tr_prefix = config.GetString("terrain.unpack_dir_prefix", "./tmp_terrain_");

        for(int i=0; i<4; ++i) {
            std::string t_dir = std::filesystem::absolute(t_prefix + std::to_string(i)).string();
            std::string tr_dir = std::filesystem::absolute(tr_prefix + std::to_string(i)).string();
            ensure_dir(t_dir);
            ensure_dir(tr_dir);
            tires.push_back(new FmuHelper("TireFMU_" + std::to_string(i), tire_fmu_file, t_dir));
            terrains.push_back(new FmuHelper("TerrainFMU_" + std::to_string(i), terrain_fmu_file, tr_dir));
        }

        vehicle_fmu.Instantiate();
        powertrain_fmu.Instantiate();
        driver_fmu.Instantiate();
        for(auto t : tires) t->Instantiate();
        for(auto t : terrains) t->Instantiate();

        // ---------------------------------------------------------------------
        // 2. Setup Parameters
        // ---------------------------------------------------------------------
        std::cout << "Setting up parameters..." << std::endl;
        
        // Helper to set params from config map
        auto set_params_from_config = [&](FmuHelper& fmu, const std::string& config_root) {
            // Apply step step_size globally or per component? 
            // Demo requirement: usually they share step size or specified.
            // Using global step_size if not specified
             fmu.SetVariable("step_size", config.GetDouble(config_root + ".parameters.step_size", step_size));

            auto val = config.Get(config_root + ".parameters");
            if (val.type == MiniJSON::Type::Object) {
                for(auto& [key, v] : val.o_val) {
                    if (key == "step_size") continue; // handled above with fallback
                    if (v.type == MiniJSON::Type::String) fmu.SetVariable(key, v.s_val);
                    else if (v.type == MiniJSON::Type::Number) fmu.SetVariable(key, v.n_val);
                    else if (v.type == MiniJSON::Type::Boolean) fmu.SetVariable(key, v.b_val);
                }
            }
        };

        set_params_from_config(vehicle_fmu, "vehicle");
        set_params_from_config(powertrain_fmu, "powertrain");
        set_params_from_config(driver_fmu, "driver");

        for(auto t : tires) set_params_from_config(*t, "tire");
        for(auto t : terrains) {
            // Special handling for Terrain FMU which might not have standard parameters exposed yet?
            // Re-enabling based on user request to be configurable.
            set_params_from_config(*t, "terrain");
        }

        // ---------------------------------------------------------------------
        // 3. Initialize
        // ---------------------------------------------------------------------
        std::cout << "Initializing..." << std::endl;

        vehicle_fmu.SetupExperiment(start_time, t_end);
        powertrain_fmu.SetupExperiment(start_time, t_end);
        driver_fmu.SetupExperiment(start_time, t_end);
        for(auto t : tires) t->SetupExperiment(start_time, t_end);
        for(auto t : terrains) t->SetupExperiment(start_time, t_end);

        vehicle_fmu.EnterInitializationMode();
        powertrain_fmu.EnterInitializationMode();
        driver_fmu.EnterInitializationMode();
        for(auto t : tires) t->EnterInitializationMode();
        for(auto t : terrains) t->EnterInitializationMode();

        // Initial location exchange (Driver -> Vehicle)
        {
            double init_loc[3];
            double init_yaw;
            GetVecVariable(driver_fmu, "init_loc", init_loc);
            driver_fmu.GetVariable("init_yaw", init_yaw);
            
            SetVecVariable(vehicle_fmu, "init_loc", init_loc);
            vehicle_fmu.SetVariable("init_yaw", init_yaw);
        }

        vehicle_fmu.ExitInitializationMode();
        powertrain_fmu.ExitInitializationMode();
        driver_fmu.ExitInitializationMode();
        for(auto t : tires) t->ExitInitializationMode();
        for(auto t : terrains) t->ExitInitializationMode();

        // ---------------------------------------------------------------------
        // 4. Simulation Loop
        // ---------------------------------------------------------------------
        std::cout << "Starting simulation loop..." << std::endl;
        
        double time = start_time;
        std::string wheel_ids[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

        while (time < t_end) {
            // --- Driver Control ---

            double steering, throttle, braking;
            driver_fmu.GetVariable("steering", steering);
            driver_fmu.GetVariable("throttle", throttle);
            driver_fmu.GetVariable("braking", braking);

            vehicle_fmu.SetVariable("steering", steering);
            vehicle_fmu.SetVariable("throttle", throttle);
            vehicle_fmu.SetVariable("braking", braking);
            powertrain_fmu.SetVariable("throttle", throttle);

            // --- Vehicle State -> Driver ---
            // "ref_frame" is a FrameMoving.
            // FMUs expose this as ref_frame.pos, ref_frame.rot, ref_frame.pos_dt, ref_frame.rot_dt
            
            double ref_pos[3], ref_rot[4], ref_pos_dt[3], ref_rot_dt[4];
            GetVecVariable(vehicle_fmu, "ref_frame.pos", ref_pos);
            GetQuatVariable(vehicle_fmu, "ref_frame.rot", ref_rot);
            GetVecVariable(vehicle_fmu, "ref_frame.pos_dt", ref_pos_dt);
            GetQuatVariable(vehicle_fmu, "ref_frame.rot_dt", ref_rot_dt);

            SetVecVariable(driver_fmu, "ref_frame.pos", ref_pos);
            SetQuatVariable(driver_fmu, "ref_frame.rot", ref_rot);
            SetVecVariable(driver_fmu, "ref_frame.pos_dt", ref_pos_dt);
            SetQuatVariable(driver_fmu, "ref_frame.rot_dt", ref_rot_dt);


            // --- Powertrain <-> Vehicle ---
            double driveshaft_torque, driveshaft_speed;
            powertrain_fmu.GetVariable("driveshaft_torque", driveshaft_torque);
            vehicle_fmu.SetVariable("driveshaft_torque", driveshaft_torque);

            vehicle_fmu.GetVariable("driveshaft_speed", driveshaft_speed);
            powertrain_fmu.SetVariable("driveshaft_speed", driveshaft_speed);

            // --- Tires & Terrains ---
            for(int i=0; i<4; ++i) {
                // Vehicle -> Tire
                double w_pos[3], w_rot[4], w_lin_vel[3], w_ang_vel[3];
                GetVecVariable(vehicle_fmu, wheel_ids[i] + ".pos", w_pos);
                GetQuatVariable(vehicle_fmu, wheel_ids[i] + ".rot", w_rot);
                GetVecVariable(vehicle_fmu, wheel_ids[i] + ".lin_vel", w_lin_vel);
                GetVecVariable(vehicle_fmu, wheel_ids[i] + ".ang_vel", w_ang_vel);

                SetVecVariable(*tires[i], "wheel_state.pos", w_pos);
                SetQuatVariable(*tires[i], "wheel_state.rot", w_rot);
                SetVecVariable(*tires[i], "wheel_state.lin_vel", w_lin_vel);
                SetVecVariable(*tires[i], "wheel_state.ang_vel", w_ang_vel);

                // Tire -> Vehicle
                double t_point[3], t_force[3], t_moment[3];
                GetVecVariable(*tires[i], "wheel_load.point", t_point);
                GetVecVariable(*tires[i], "wheel_load.force", t_force);
                GetVecVariable(*tires[i], "wheel_load.moment", t_moment);

                SetVecVariable(vehicle_fmu, wheel_ids[i] + ".point", t_point);
                SetVecVariable(vehicle_fmu, wheel_ids[i] + ".force", t_force);
                SetVecVariable(vehicle_fmu, wheel_ids[i] + ".moment", t_moment);

                // Tire -> Terrain
                double query_point[3];
                GetVecVariable(*tires[i], "query_point", query_point);
                SetVecVariable(*terrains[i], "query_point", query_point);

                // Step Terrain
                terrains[i]->DoStep(time, step_size);

                // Terrain -> Tire
                double height, mu, normal[3];
                terrains[i]->GetVariable("height", height);
                terrains[i]->GetVariable("mu", mu);
                GetVecVariable(*terrains[i], "normal", normal);
                
                tires[i]->SetVariable("terrain_height", height);
                tires[i]->SetVariable("terrain_mu", mu);
                SetVecVariable(*tires[i], "terrain_normal", normal);
            }

            // --- Advance Steps ---
            /*
            auto status_vehicle = vehicle_fmu.DoStep(time, step, fmi2True);
            auto status_powertrain = powertrain_fmu.DoStep(time, step, fmi2True);
            auto status_driver = driver_fmu.DoStep(time, step, fmi2True);
            // Terrain already stepped
            */
            if(vehicle_fmu.DoStep(time, step_size) != fmi2_status_ok) break;
            if(powertrain_fmu.DoStep(time, step_size) != fmi2_status_ok) break;
            if(driver_fmu.DoStep(time, step_size) != fmi2_status_ok) break;
            
            for(auto t : tires) {
                if(t->DoStep(time, step_size) != fmi2_status_ok) break;
            }

            time += step_size;
            
            if (static_cast<int>(time * 1000) % 100 == 0) { // Print every 0.1s
                 // Use ref_pos_dt[0] as speed approx or sqrt(v*v)
                 double speed = std::sqrt(ref_pos_dt[0]*ref_pos_dt[0] + ref_pos_dt[1]*ref_pos_dt[1] + ref_pos_dt[2]*ref_pos_dt[2]);
                 std::cout << "Time: " << time << " Speed: " << speed << " Throttle: " << throttle << std::endl;
            }
        }

        std::cout << "Simulation finished at time " << time << std::endl;

        // Cleanup
        for(auto t : tires) delete t;
        for(auto t : terrains) delete t;

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
