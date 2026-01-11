#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <array>
#include <iomanip>
#include <chrono>
#include "FmuHelper.h"
#include "OsiHelper.h"
#include "DemoConfiguration.h"

// OSI Ptrs
#include "osi_sensorview.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_trafficupdate.pb.h"

// Helper for vector/quat names since they are expanded in FMI
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

int main(int argc, char* argv[]) {
    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    DemoConfiguration config;
    if (!config.Load("demo_config.json")) {
        std::cerr << "Error: Could not load demo_config.json" << std::endl;
        return 1;
    }

    double step_size = config.GetDouble("simulation.step_size", 1e-2);
    int chrono_substeps = (int)config.GetDouble("simulation.chrono_substeps", 1.0);
    if (chrono_substeps < 1) chrono_substeps = 1;

    double start_time = config.GetDouble("simulation.start_time", 0.0);
    double t_end = config.GetDouble("simulation.end_time", 20.0);
    
    // FMU Filenames & Paths
    auto get_abs_path = [&](const std::string& key) {
        std::string p = config.GetString(key, "");
        if(p.empty()) throw std::runtime_error("Missing config: " + key);
        return std::filesystem::absolute(p).string();
    };

    std::string esmini_fmu_file = get_abs_path("esmini.fmu_path");
    std::string drivecontroller_fmu_file = get_abs_path("drivecontroller.fmu_path");
    std::string vehicle_fmu_file = get_abs_path("vehicle.fmu_path");
    std::string powertrain_fmu_file = get_abs_path("powertrain.fmu_path");
    std::string tire_fmu_file = get_abs_path("tire.fmu_path");
    std::string terrain_fmu_file = get_abs_path("terrain.fmu_path");

    // Check existence of critical FMUs
    if (!std::filesystem::exists(esmini_fmu_file)) {
        std::cerr << "Error: esmini FMU not found at " << esmini_fmu_file << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(drivecontroller_fmu_file)) {
        std::cerr << "Error: DriveController FMU not found at " << drivecontroller_fmu_file << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(vehicle_fmu_file)) {
        std::cerr << "Error: Vehicle FMU not found at " << vehicle_fmu_file << std::endl;
        return 1;
    }

    try {
        // ---------------------------------------------------------------------
        // 1. Instantiate FMUs
        // ---------------------------------------------------------------------
        std::cout << "Instantiating FMUs..." << std::endl;
        
        auto ensure_dir = [](const std::string& path) {
            if (!std::filesystem::exists(path)) std::filesystem::create_directories(path);
        };

        std::string esmini_unpack = std::filesystem::absolute(config.GetString("esmini.unpack_dir", "./tmp_esmini")).string();
        std::string dc_unpack = std::filesystem::absolute(config.GetString("drivecontroller.unpack_dir", "./tmp_dc")).string();
        std::string v_unpack = std::filesystem::absolute(config.GetString("vehicle.unpack_dir", "./tmp_vehicle")).string();
        std::string p_unpack = std::filesystem::absolute(config.GetString("powertrain.unpack_dir", "./tmp_powertrain")).string();

        ensure_dir(esmini_unpack);
        ensure_dir(dc_unpack);
        ensure_dir(v_unpack);
        ensure_dir(p_unpack);

        FmuHelper esmini_fmu("EsminiFMU", esmini_fmu_file, esmini_unpack);
        FmuHelper drivecontroller_fmu("DriveControllerFMU", drivecontroller_fmu_file, dc_unpack);
        FmuHelper vehicle_fmu("WheeledVehicleFMU", vehicle_fmu_file, v_unpack);
        FmuHelper powertrain_fmu("PowertrainFMU", powertrain_fmu_file, p_unpack);

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

        std::cout << "Instantiating esmini FMU..." << std::endl;
        esmini_fmu.Instantiate();
        
        std::cout << "Instantiating DriveController FMU..." << std::endl;
        drivecontroller_fmu.Instantiate();
        
        std::cout << "Instantiating Vehicle FMU..." << std::endl;
        vehicle_fmu.Instantiate();
        
        std::cout << "Instantiating Powertrain FMU..." << std::endl;
        powertrain_fmu.Instantiate();
        
        std::cout << "Instantiating Tire FMUs..." << std::endl;
        for(auto t : tires) t->Instantiate();
        
        std::cout << "Instantiating Terrain FMUs..." << std::endl;
        for(auto t : terrains) t->Instantiate();

        // ---------------------------------------------------------------------
        // 1.5. Delayed Initialization (Scenario-based Init)
        // ---------------------------------------------------------------------
        // We need to initialize esmini *first* to get the initial ground truth.
        // Then we extract the position of the first moving object (Host Vehicle)
        // and pass it to Chrono FMU.
        
        std::cout << "Initializing esmini to get scenario start position..." << std::endl;
        
        // Setup esmini parameters first (needed for initialization)
        esmini_fmu.SetVariable("xosc_path", config.GetString("esmini.xosc_path", ""));
        // set_params_from_config(esmini_fmu, "esmini"); 
       // Add step_size if needed, though usually fixed_timestep arg handles it
        // fmu.SetVariable("step_size", step_size); 

        // Apply config parameters to esmini before init
        auto set_params_from_config_esmini = [&](FmuHelper& fmu, const std::string& config_root) {
             auto val = config.Get(config_root + ".parameters");
             if (val.type == MiniJSON::Type::Object) {
                 for(auto& [key, v] : val.o_val) {
                     if (key == "step_size") continue;
                     if (v.type == MiniJSON::Type::String) fmu.SetVariable(key, v.s_val);
                     else if (v.type == MiniJSON::Type::Number) fmu.SetVariable(key, v.n_val);
                     else if (v.type == MiniJSON::Type::Boolean) fmu.SetVariable(key, v.b_val);
                 }
             }
        };
        set_params_from_config_esmini(esmini_fmu, "esmini");
        std::cout << "complete set_params" << std::endl;
        // Initialize esmini
        esmini_fmu.SetupExperiment(0.0, 0.0, 0.0);
        // esmini_fmu.SetupExperiment(start_time, t_end);

        std::cout << "complete setup_experiment" << std::endl;

        esmini_fmu.EnterInitializationMode();
        std::cout << "complete EnterInitializationMode" << std::endl;
        std::cerr << "[TRACE] BEFORE esmini ExitInitializationMode" << std::endl;
        esmini_fmu.ExitInitializationMode();
        std::cerr << "[TRACE] AFTER esmini ExitInitializationMode" << std::endl;


        // Get Initial OSI
        std::cout << "Extracting initial OSI from esmini..." << std::endl;
        
        int sv_lo, sv_hi, sv_sz;
        esmini_fmu.GetVariable("OSMPSensorViewOut.base.lo", sv_lo);
        esmini_fmu.GetVariable("OSMPSensorViewOut.base.hi", sv_hi);
        esmini_fmu.GetVariable("OSMPSensorViewOut.size", sv_sz);
        
        double initial_pos[3] = {0,0,0};
        double initial_rot[3] = {0,0,0}; // roll, pitch, yaw
        bool found_ego = false;

        if (sv_sz > 0) {
            void* ptr = DecodeOSMPPointer(sv_lo, sv_hi);
            osi3::SensorView sv;
            if (sv.ParseFromArray(ptr, sv_sz)) {
                if (sv.has_global_ground_truth() && sv.global_ground_truth().moving_object_size() > 0) {
                     // As per user request: Use the first moving object
                     const auto& obj = sv.global_ground_truth().moving_object(0);
                     if (obj.has_base()) {
                         initial_pos[0] = obj.base().position().x();
                         initial_pos[1] = obj.base().position().y();
                         initial_pos[2] = obj.base().position().z();
                         
                         initial_rot[0] = obj.base().orientation().roll();
                         initial_rot[1] = obj.base().orientation().pitch();
                         initial_rot[2] = obj.base().orientation().yaw();
                         
                         std::cout << "[Scenario Init] Found Ego Initial State: Pos(" 
                                   << initial_pos[0] << ", " << initial_pos[1] << ", " << initial_pos[2] << ") "
                                   << "Rot(" << initial_rot[0] << ", " << initial_rot[1] << ", " << initial_rot[2] << ")" << std::endl;
                         found_ego = true;
                     }
                }
            } else {
                 std::cerr << "[Error] Failed to parse initial OSI SensorView!" << std::endl;
            }
        } else {
             std::cerr << "[Error] Initial OSI size is 0!" << std::endl;
        }

        // ---------------------------------------------------------------------
        // 2. Setup Parameters (Chrono & others)
        // ---------------------------------------------------------------------
        std::cout << "Setting up parameters for other FMUs..." << std::endl;
        
        auto set_params_from_config = [&](FmuHelper& fmu, const std::string& config_root) {
            fmu.SetVariable("step_size", config.GetDouble(config_root + ".parameters.step_size", step_size));

            auto val = config.Get(config_root + ".parameters");
            if (val.type == MiniJSON::Type::Object) {
                for(auto& [key, v] : val.o_val) {
                    if (key == "step_size") continue;
                    if (v.type == MiniJSON::Type::String) {
                         fmu.SetVariable(key, v.s_val);
                         std::cout << "[DEBUG] Set " << key << " = " << v.s_val << " (" << config_root << ")" << std::endl;
                    }
                    else if (v.type == MiniJSON::Type::Number) {
                         fmu.SetVariable(key, v.n_val);
                         std::cout << "[DEBUG] Set " << key << " = " << v.n_val << " (" << config_root << ")" << std::endl;
                    }
                    else if (v.type == MiniJSON::Type::Boolean) {
                         fmu.SetVariable(key, v.b_val);
                         std::cout << "[DEBUG] Set " << key << " = " << (v.b_val ? "true" : "false") << " (" << config_root << ")" << std::endl;
                    }
                }
            }
        };

        // set_params_from_config(esmini_fmu, "esmini"); // Already done
        set_params_from_config(drivecontroller_fmu, "drivecontroller");
        set_params_from_config(vehicle_fmu, "vehicle");
        set_params_from_config(powertrain_fmu, "powertrain");

        for(auto t : tires) set_params_from_config(*t, "tire");
        for(auto t : terrains) set_params_from_config(*t, "terrain");

        // [scenario-init] Apply extracted position to Vehicle FMU
        if (found_ego) {
             std::cout << "[Scenario Init] Overriding Vehicle FMU initial state from scenario." << std::endl;
             std::cout << "  Position: " << initial_pos[0] << ", " << initial_pos[1] << ", " << initial_pos[2] << std::endl;
             std::cout << "  Yaw:      " << initial_rot[2] << std::endl;

             vehicle_fmu.SetVariable("init_loc.x", initial_pos[0]);
             vehicle_fmu.SetVariable("init_loc.y", initial_pos[1]);
             vehicle_fmu.SetVariable("init_loc.z", initial_pos[2]);
             vehicle_fmu.SetVariable("init_yaw", initial_rot[2]); 
        }

        // ---------------------------------------------------------------------
        // 3. Initialize (Enter/Exit Init Mode) - excluding esmini
        // ---------------------------------------------------------------------
        std::cout << "Initializing other FMUs..." << std::endl;
        
        // Esmini is skipped here because it was initialized earlier.

        drivecontroller_fmu.SetupExperiment(start_time, t_end);
        vehicle_fmu.SetupExperiment(start_time, t_end);
        powertrain_fmu.SetupExperiment(start_time, t_end);
        for(auto t : tires) t->SetupExperiment(start_time, t_end);
        for(auto t : terrains) t->SetupExperiment(start_time, t_end);

        drivecontroller_fmu.EnterInitializationMode();
        vehicle_fmu.EnterInitializationMode();
        powertrain_fmu.EnterInitializationMode();
        for(auto t : tires) t->EnterInitializationMode();
        for(auto t : terrains) t->EnterInitializationMode();
        
        drivecontroller_fmu.ExitInitializationMode();
        vehicle_fmu.ExitInitializationMode();
        powertrain_fmu.ExitInitializationMode();
        for(auto t : tires) t->ExitInitializationMode();
        for(auto t : terrains) t->ExitInitializationMode();
        std::cout << "[DEBUG] All init done." << std::endl;

        // [Post-Init Check] Read back the specific coordinates to verify override
        double init_check_pos[3];
        GetVecVariable(vehicle_fmu, "ref_frame.pos", init_check_pos);
        std::cout << "[Chrono Init Result] Pos: (" 
                  << init_check_pos[0] << ", " 
                  << init_check_pos[1] << ", " 
                  << init_check_pos[2] << ")" << std::endl;

        // ---------------------------------------------------------------------
        // 4. Simulation Loop
        // ---------------------------------------------------------------------
        std::cout << "Starting simulation loop..." << std::endl;
        std::cout << "Step size: " << step_size << " s, End time: " << t_end << " s" << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        
        double time = start_time;
        std::string wheel_ids[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};
        int step_count = 0;

        // [Feedback] State variables
        osi3::TrafficUpdate current_tu;
        osi3::MovingObject stored_ego_obj; // Template object
        std::string tu_buffer;
        tu_buffer.reserve(64000); 
        bool ego_found_in_dc = false;
    uint64_t found_ego_id = 0; // Store detected ID

        while (time < t_end) {
            // --- esmini -> DriveController (OSI SensorView) ---
            int osi_sv_lo, osi_sv_hi, osi_sv_size;
            esmini_fmu.GetVariable("OSMPSensorViewOut.base.lo", osi_sv_lo);
            esmini_fmu.GetVariable("OSMPSensorViewOut.base.hi", osi_sv_hi);
            esmini_fmu.GetVariable("OSMPSensorViewOut.size", osi_sv_size);

            std::cout << "[DEBUG] Step " << time << ": OSI size=" << osi_sv_size << std::endl;

            // Direct pointer transfer (same process)
            drivecontroller_fmu.SetVariable("OSI_SensorView_In_BaseLo", osi_sv_lo);
            drivecontroller_fmu.SetVariable("OSI_SensorView_In_BaseHi", osi_sv_hi);
            drivecontroller_fmu.SetVariable("OSI_SensorView_In_Size", osi_sv_size);

            // Debug: Decode pointer to verify (optional)
            if (osi_sv_size > 0 && step_count % 100 == 0) {
                void* osi_ptr = DecodeOSMPPointer(osi_sv_lo, osi_sv_hi);
                std::cout << "[DEBUG] OSI SensorView pointer: " << osi_ptr 
                          << ", size: " << osi_sv_size << " bytes" << std::endl;
            }

            // --- Step DriveController ---
            std::cerr << "[TRACE] Stepping DriveController..." << std::endl;
            if(drivecontroller_fmu.DoStep(time, step_size) != fmi2_status_ok) {
                std::cerr << "DriveController FMU step failed at time " << time << std::endl;
                break;
            }
            std::cout << "[DEBUG] DriveController Step OK" << std::endl;

            std::cout << "[DEBUG] DriveController Step OK" << std::endl;

            // [Feedback] 1. Identify Ego from DC Output
            if (!ego_found_in_dc) {
                int dc_sv_lo=0, dc_sv_hi=0, dc_sv_sz=0;
                // Note: Assuming these variables exist on the FMU based on user instruction
                drivecontroller_fmu.GetVariable("OSI_SensorView_Out_BaseLo", dc_sv_lo);
                drivecontroller_fmu.GetVariable("OSI_SensorView_Out_BaseHi", dc_sv_hi);
                drivecontroller_fmu.GetVariable("OSI_SensorView_Out_Size", dc_sv_sz);
                
                if (dc_sv_sz > 0) {
                    void* ptr = DecodeOSMPPointer(dc_sv_lo, dc_sv_hi);
                    osi3::SensorView dc_sv;
                    // Use ParseFromArray with caution on pointer validity
                    if (dc_sv.ParseFromArray(ptr, dc_sv_sz)) {
                        if (dc_sv.has_global_ground_truth() && dc_sv.global_ground_truth().moving_object_size() > 0) {
                            const auto& ego_obj = dc_sv.global_ground_truth().moving_object(0);
                            // Copy ID and Object to TrafficUpdate (Base for updates)
                            stored_ego_obj.CopyFrom(ego_obj); // [RESTORED] Copy useful metadata
                            found_ego_id = ego_obj.id().value();
                            // copy to current_tu for consistency/logging if needed, though cleared downstream
                            current_tu.add_update()->CopyFrom(ego_obj); 
                            found_ego_id = ego_obj.id().value();
                            std::cout << "[Feedback] Found Ego ID from DC: " << ego_obj.id().value() << std::endl;
                            ego_found_in_dc = true;
                        }
                    }
                }
            }

            // --- DriveController -> Vehicle (Control Inputs) ---
            double throttle, brake, steering;
            // std::cout << "[DEBUG] Getting DriveController outputs..." << std::endl;
            drivecontroller_fmu.GetVariable("Throttle", throttle);
            drivecontroller_fmu.GetVariable("Brake", brake);
            drivecontroller_fmu.GetVariable("Steering", steering);
            // std::cout << "[DEBUG] Outputs: T=" << throttle << " B=" << brake << " S=" << steering << std::endl;

            // std::cout << "[DEBUG] Setting Vehicle inputs..." << std::endl;
            vehicle_fmu.SetVariable("steering", steering);
            vehicle_fmu.SetVariable("throttle", throttle);
            vehicle_fmu.SetVariable("braking", brake);
            powertrain_fmu.SetVariable("throttle", throttle);
            // std::cout << "[DEBUG] Vehicle inputs set." << std::endl;

            // --- Chrono Co-simulation (Sub-stepping) ---
            double chrono_step_size = step_size / chrono_substeps;
            double current_chrono_time = time;

            for (int sub = 0; sub < chrono_substeps; ++sub) {
                // Powertrain <-> Vehicle
                double driveshaft_torque, driveshaft_speed;
                powertrain_fmu.GetVariable("driveshaft_torque", driveshaft_torque);
                vehicle_fmu.SetVariable("driveshaft_torque", driveshaft_torque);

                vehicle_fmu.GetVariable("driveshaft_speed", driveshaft_speed);
                powertrain_fmu.SetVariable("driveshaft_speed", driveshaft_speed);

                // Tires & Terrains
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
                    terrains[i]->DoStep(current_chrono_time, chrono_step_size);

                    // Terrain -> Tire
                    double height, mu, normal[3];
                    terrains[i]->GetVariable("height", height);
                    terrains[i]->GetVariable("mu", mu);
                    GetVecVariable(*terrains[i], "normal", normal);
                    
                    tires[i]->SetVariable("terrain_height", height);
                    tires[i]->SetVariable("terrain_mu", mu);
                    SetVecVariable(*tires[i], "terrain_normal", normal);
                }

                // --- Step FMUs ---
                if(vehicle_fmu.DoStep(current_chrono_time, chrono_step_size) != fmi2_status_ok) {
                    std::cerr << "Vehicle FMU step failed at sub-step time " << current_chrono_time << std::endl;
                    // In a real app handle error, here we could break
                }
                if(powertrain_fmu.DoStep(current_chrono_time, chrono_step_size) != fmi2_status_ok) {
                    std::cerr << "Powertrain FMU step failed at sub-step time " << current_chrono_time << std::endl;
                }
                
                for(auto t : tires) {
                    t->DoStep(current_chrono_time, chrono_step_size);
                }
                
                current_chrono_time += chrono_step_size;
            }

            // [Feedback] 2. Update TrafficUpdate with minimal construction
            if (ego_found_in_dc) {
                double c_pos[3];
                GetVecVariable(vehicle_fmu, "ref_frame.pos", c_pos);

                // Update TrafficUpdate - Full Construction
                current_tu.Clear();
                current_tu.mutable_timestamp()->set_seconds((int64_t)time);
                current_tu.mutable_timestamp()->set_nanos((int)((time - (int64_t)time) * 1e9));

                auto* update_obj = current_tu.add_update();
                update_obj->CopyFrom(stored_ego_obj); // Start with full copy
                
                auto* base = update_obj->mutable_base();
                base->mutable_position()->set_x(c_pos[0]);
                base->mutable_position()->set_y(c_pos[1]);
                base->mutable_position()->set_z(c_pos[2]);

                // Serialize
                tu_buffer.clear();
                current_tu.SerializeToString(&tu_buffer);

                // Send to esmini
                int32_t tu_lo, tu_hi;
                EncodeOSMPPointer(const_cast<char*>(tu_buffer.data()), tu_lo, tu_hi);
                int32_t tu_sz = static_cast<int32_t>(tu_buffer.size());

                esmini_fmu.SetVariable("OSMPTrafficUpdateIn.base.lo", tu_lo);
                esmini_fmu.SetVariable("OSMPTrafficUpdateIn.base.hi", tu_hi);
                esmini_fmu.SetVariable("OSMPTrafficUpdateIn.size", tu_sz);
            }

            std::cerr << "[TRACE] Stepping Esmini..." << std::endl;
            if(esmini_fmu.DoStep(time, step_size) != fmi2_status_ok) {
                std::cerr << "Esmini FMU step failed at time " << time << std::endl;
                break;
            }

            // --- Get and Display Chrono Vehicle State ---
            double ref_pos[3], ref_rot[4], ref_pos_dt[3];
            GetVecVariable(vehicle_fmu, "ref_frame.pos", ref_pos);
            GetQuatVariable(vehicle_fmu, "ref_frame.rot", ref_rot);
            GetVecVariable(vehicle_fmu, "ref_frame.pos_dt", ref_pos_dt);

            double speed = std::sqrt(ref_pos_dt[0]*ref_pos_dt[0] + 
                                     ref_pos_dt[1]*ref_pos_dt[1] + 
                                     ref_pos_dt[2]*ref_pos_dt[2]);

            // Print every 0.1 second (10Hz)
            if (step_count % static_cast<int>(0.1 / step_size) == 0) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "[Chrono Sim] "
                          << "Time: " << std::setw(6) << time << " s | "
                          << "Pos: (" << std::setw(7) << ref_pos[0] << ", " 
                          << std::setw(7) << ref_pos[1] << ", " 
                          << std::setw(7) << ref_pos[2] << ") | "
                          << "Speed: " << std::setw(6) << speed << " m/s | "
                          << "Throttle: " << std::setw(5) << throttle << " | "
                          << "Brake: " << std::setw(5) << brake << " | "
                          << "Steering: " << std::setw(6) << steering
                          << std::endl;
            }

            time += step_size;
            step_count++;
        }

        std::cout << std::string(80, '=') << std::endl;
        std::cout << "Simulation finished at time " << time << " s" << std::endl;

        // Cleanup
        for(auto t : tires) delete t;
        for(auto t : terrains) delete t;

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
