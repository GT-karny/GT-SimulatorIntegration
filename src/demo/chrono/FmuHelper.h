#pragma once

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fmilib.h>

class FmuHelper {
public:
    FmuHelper(const std::string& instanceName, const std::string& fmuPath, const std::string& unzipDir);
    ~FmuHelper();

    // Setup and Initialization
    void Instantiate(bool visible = false, bool loggingOn = false);
    void SetupExperiment(double startTime, double stopTime, double tolerance = 0.0);
    void EnterInitializationMode();
    void ExitInitializationMode();

    // Simulation Step
    fmi2_status_t DoStep(double currentCommunicationPoint, double communicationStepSize, bool noSetFMUStatePriorToCurrentPoint = true);

    // Variable Access
    bool SetVariable(const std::string& name, double value);
    bool SetVariable(const std::string& name, int value);
    bool SetVariable(const std::string& name, bool value);
    bool SetVariable(const std::string& name, const std::string& value);

    bool GetVariable(const std::string& name, double& value);
    bool GetVariable(const std::string& name, int& value);
    bool GetVariable(const std::string& name, bool& value);
    bool GetVariable(const std::string& name, std::string& value);

    // Helpers
    std::string GetVersion() const;
    std::string GetTypesPlatform() const;

private:
    void ParseModelDescription();
    fmi2_value_reference_t GetValueReference(const std::string& name);

    std::string m_instanceName;
    std::string m_fmuPath;
    std::string m_unzipDir;

    fmi_import_context_t* m_context = nullptr;
    fmi2_import_t* m_fmu = nullptr;
    fmi2_callback_functions_t m_callbacks;
    jm_callbacks m_jmCallbacks;

    std::map<std::string, fmi2_import_variable_t*> m_variableMap;
    std::map<std::string, fmi2_value_reference_t> m_vrCache;
};
