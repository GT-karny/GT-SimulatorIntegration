#include "FmuHelper.h"
#include <stdexcept>
#include <filesystem>

#include <filesystem>
#include <FMI/fmi_zip_unzip.h>

// Callback functions for FMI
static void fmiLogger(fmi2_component_environment_t componentEnvironment,
                      fmi2_string_t instanceName,
                      fmi2_status_t status,
                      fmi2_string_t category,
                      fmi2_string_t message, ...) {
    va_list args;
    va_start(args, message);
    printf("Logger: %s: %s\n", instanceName, message);
    va_end(args);
}

static void* fmiAllocateMemory(size_t nobj, size_t size) {
    return calloc(nobj, size);
}

static void fmiFreeMemory(void* obj) {
    free(obj);
}

static void jmLogger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {
    printf("Logger: module %s: %s\n", module, message);
}

FmuHelper::FmuHelper(const std::string& instanceName, const std::string& fmuPath, const std::string& unzipDir)
    : m_instanceName(instanceName), m_fmuPath(fmuPath), m_unzipDir(unzipDir) {

    // Setup JM callbacks
    m_jmCallbacks.malloc = malloc;
    m_jmCallbacks.calloc = calloc;
    m_jmCallbacks.realloc = realloc;
    m_jmCallbacks.free = free;
    m_jmCallbacks.logger = jmLogger;
    m_jmCallbacks.log_level = jm_log_level_warning;
    m_jmCallbacks.context = 0;

    // Setup FMI2 callbacks
    m_callbacks.logger = fmiLogger;
    m_callbacks.allocateMemory = fmiAllocateMemory;
    m_callbacks.freeMemory = fmiFreeMemory;
    m_callbacks.stepFinished = nullptr;
    m_callbacks.componentEnvironment = nullptr;

    m_context = fmi_import_allocate_context(&m_jmCallbacks);

    // Unzip FMU
    if (fmi_zip_unzip(m_fmuPath.c_str(), m_unzipDir.c_str(), &m_jmCallbacks) != jm_status_success) {
        throw std::runtime_error("Failed to unzip FMU: " + m_fmuPath);
    }

    // Parse model description
    m_fmu = fmi2_import_parse_xml(m_context, m_unzipDir.c_str(), 0);
    if (!m_fmu) {
        throw std::runtime_error("Failed to parse model description: " + m_fmuPath);
    }

    // Check FMI version (optional but good practice)
    // ...

    // Load DLL
    if (fmi2_import_create_dllfmu(m_fmu, fmi2_fmu_kind_cs, &m_callbacks) != jm_status_success) {
        throw std::runtime_error("Failed to load DLL for FMU: " + m_fmuPath);
    }
}

FmuHelper::~FmuHelper() {
    if (m_fmu) {
        fmi2_import_terminate(m_fmu);
        fmi2_import_free_instance(m_fmu);
        fmi2_import_destroy_dllfmu(m_fmu);
        fmi2_import_free(m_fmu);
    }
    if (m_context) {
        fmi_import_free_context(m_context);
    }
}

void FmuHelper::Instantiate(bool visible, bool loggingOn) {
    if (fmi2_import_instantiate(m_fmu, m_instanceName.c_str(), fmi2_cosimulation, nullptr, visible) != jm_status_success) {
        throw std::runtime_error("Failed to instantiate FMU: " + m_instanceName);
    }
}

void FmuHelper::SetupExperiment(double startTime, double stopTime, double tolerance) {
    fmi2_import_setup_experiment(m_fmu, fmi2_true, tolerance, startTime, fmi2_true, stopTime);
}

void FmuHelper::EnterInitializationMode() {
    if (fmi2_import_enter_initialization_mode(m_fmu) != fmi2_status_ok) {
        throw std::runtime_error("Failed to enter initialization mode: " + m_instanceName);
    }
}

void FmuHelper::ExitInitializationMode() {
    if (fmi2_import_exit_initialization_mode(m_fmu) != fmi2_status_ok) {
        throw std::runtime_error("Failed to exit initialization mode: " + m_instanceName);
    }
}

fmi2_status_t FmuHelper::DoStep(double currentCommunicationPoint, double communicationStepSize, bool noSetFMUStatePriorToCurrentPoint) {
    return fmi2_import_do_step(m_fmu, currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPoint ? fmi2_true : fmi2_false);
}

fmi2_value_reference_t FmuHelper::GetValueReference(const std::string& name) {
    if (m_vrCache.find(name) != m_vrCache.end()) {
        return m_vrCache[name];
    }

    fmi2_import_variable_t* var = fmi2_import_get_variable_by_name(m_fmu, name.c_str());
    if (!var) {
        // Log warning or throw? For now just return a dummy or throw
        std::cerr << "Warning: Variable not found: " << name << " in " << m_instanceName << std::endl;
        return 999999; // Invalid VR
    }
    fmi2_value_reference_t vr = fmi2_import_get_variable_vr(var);
    m_vrCache[name] = vr;
    return vr;
}

bool FmuHelper::SetVariable(const std::string& name, double value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    return fmi2_import_set_real(m_fmu, &vr, 1, &value) == fmi2_status_ok;
}

bool FmuHelper::SetVariable(const std::string& name, int value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    return fmi2_import_set_integer(m_fmu, &vr, 1, &value) == fmi2_status_ok;
}

bool FmuHelper::SetVariable(const std::string& name, bool value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    fmi2_boolean_t val = value ? fmi2_true : fmi2_false;
    return fmi2_import_set_boolean(m_fmu, &vr, 1, &val) == fmi2_status_ok;
}

bool FmuHelper::SetVariable(const std::string& name, const std::string& value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    const char* val = value.c_str();
    return fmi2_import_set_string(m_fmu, &vr, 1, &val) == fmi2_status_ok;
}

bool FmuHelper::GetVariable(const std::string& name, double& value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    return fmi2_import_get_real(m_fmu, &vr, 1, &value) == fmi2_status_ok;
}

bool FmuHelper::GetVariable(const std::string& name, int& value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    return fmi2_import_get_integer(m_fmu, &vr, 1, &value) == fmi2_status_ok;
}

bool FmuHelper::GetVariable(const std::string& name, bool& value) {
    fmi2_value_reference_t vr = GetValueReference(name);
    fmi2_boolean_t val;
    bool success = fmi2_import_get_boolean(m_fmu, &vr, 1, &val) == fmi2_status_ok;
    value = (val == fmi2_true);
    return success;
}

bool FmuHelper::GetVariable(const std::string& name, std::string& value) {
     // FMI 2.0 string getting is a bit more complex (needs buffer management sometimes depending on impl),
     // but FMILib abstracts it slightly.
     // WARNING: fmi2_import_get_string returns a pointer that might be managed by the FMU. Use cautiously.
    fmi2_value_reference_t vr = GetValueReference(name);
    fmi2_string_t val;
    bool success = fmi2_import_get_string(m_fmu, &vr, 1, &val) == fmi2_status_ok;
    if(success) value = val;
    return success;
}

std::string FmuHelper::GetVersion() const {
    return fmi2_import_get_version(m_fmu);
}

std::string FmuHelper::GetTypesPlatform() const {
    return fmi2_import_get_types_platform(m_fmu);
}
