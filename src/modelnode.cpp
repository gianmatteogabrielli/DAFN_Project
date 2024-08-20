#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper_cpp/modelnode.hpp"


SimulinkModelNode::SimulinkModelNode()
: Node("simulink_modelNode"){

    RCLCPP_INFO(this->get_logger(), "SIMULINK NODE");
    // Inizializza la libreria Simulink
    simulinkLibraryName = "/home/neo/workspace/src/cpp/simulink_wrapper_cpp/mylibs/libdigitalfilter.so";
    symbol_prefix_ = "libdigitalfilter_";

    if (!Initialize(simulinkLibraryName, symbol_prefix_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize the Simulink model");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Initialize the Simulink model");
    }

    if(!Setup())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup the Simulink model");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Setup the Simulink model has been done succesfully");
    }

    subscriber_node = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "simulink_input", 10, std::bind(&SimulinkModelNode::topic_callback, this, std::placeholders::_1));

    publisher_node = this->create_publisher<std_msgs::msg::Float64MultiArray>("simulink_output", 10);

}

void SimulinkModelNode::topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        auto transformed_message = std_msgs::msg::Float64MultiArray();
        for (const auto &value : msg->data)
        {
            transformed_message.data.push_back(value + 1.0);
        }

        publisher_node->publish(transformed_message);

    // Logica per interagire con il modello Simulink
}


bool SimulinkModelNode::ScanTunableParameters(const rtwCAPI_ModelMappingStaticInfo* mmi_address_)
{
    RCLCPP_INFO(this->get_logger(), "SCAN TUNABLE PARAMETERS FUNCTION");
    uint32_T       nOfParams = 0u;
    std::string paramName;
    uint32_T      addrIdx;
    uint16_T      dataTypeIdx;
    uint16_T       numElements;
    uint16_T       dataTypeSize;
    uint8_T        slDataID;
    void*        paramAddress;
    uint32_T       SUBdimArrayIdx;
    uint16_T       SUBdimIdx;
    uint8_T        SUBnumDims;

    bool ok (mmi_address_ != NULL);

    // Populating C API data structure pointers of the class from mmi
    modelParams = rtwCAPI_GetModelParametersFromStaticMap(mmi_address_);
    ok = ( (modelParams != NULL) && ok);

    dataTypeMap = rtwCAPI_GetDataTypeMapFromStaticMap(mmi_address_);
    ok = ( (dataTypeMap != NULL) && ok);

    elementMap = rtwCAPI_GetElementMapFromStaticMap(mmi_address_);
    ok = ( (elementMap != NULL) && ok);

    dimMap   = rtwCAPI_GetDimensionMapFromStaticMap(mmi_address_);
    ok = ( (dimMap != NULL) && ok);

    dimArray = rtwCAPI_GetDimensionArrayFromStaticMap(mmi_address_);
    ok = ( (dimArray != NULL) && ok);

    nOfParams = rtwCAPI_GetNumModelParametersFromStaticMap(mmi_address_);
    RCLCPP_INFO(this->get_logger(), "SCAN TUNABLE PARAMETERS FUNCTION: Number of Parameters: %u", nOfParams);

    for(uint16_T paramIdx = 0u; (paramIdx < nOfParams) && ok; paramIdx++)
    {

        dataTypeIdx  = rtwCAPI_GetModelParameterDataTypeIdx(modelParams, paramIdx); // Index into the data type in rtwCAPI_DataTypeMap
        paramName    = rtwCAPI_GetModelParameterName(modelParams, paramIdx);        // Name of the parameter
        slDataID     = rtwCAPI_GetDataTypeSLId(dataTypeMap, dataTypeIdx);           // Simulink type from data type map
        numElements  = rtwCAPI_GetDataTypeNumElements(dataTypeMap,dataTypeIdx);     // Number of elements of the strucutre data type
        dataTypeSize = rtwCAPI_GetDataTypeSize(dataTypeMap,dataTypeIdx);            // Size of the datatype in bytes, WARNING: 16 bits maximum !!
    }
    return true;
}


bool SimulinkModelNode::ScanRootIO(const rtwCAPI_ModelMappingStaticInfo* mmi_address_, const rtwCAPI_Signals* mode)
{
    RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION");

    uint32_T       nOfSignals = 0u;
    std::string sigName;
    uint16_T       dataTypeIdx;
    uint8_T       slDataID;
    uint16_T      numElements;
    uint16_T       dataTypeSize;
    uint32_T       addrIdx;
    void*        sigAddress;

    bool ok = (mmi_address_ != NULL);

    // Populating C API data structure pointers of the class from mmi
    rootInputs = rtwCAPI_GetRootInputsFromStaticMap(mmi_address_);
    ok = ( (rootInputs != NULL) && ok );

    rootOutputs = rtwCAPI_GetRootOutputsFromStaticMap(mmi_address_);
    ok = ( (rootOutputs != NULL) && ok );

    dataTypeMap = rtwCAPI_GetDataTypeMapFromStaticMap(mmi_address_);
    ok = ( (dataTypeMap != NULL) && ok );

    elementMap = rtwCAPI_GetElementMapFromStaticMap(mmi_address_);
    ok = ( (elementMap != NULL) && ok );

    dimMap = rtwCAPI_GetDimensionMapFromStaticMap(mmi_address_);
    ok = ( (dimMap != NULL) && ok );

    dimArray = rtwCAPI_GetDimensionArrayFromStaticMap(mmi_address_);
    ok = ( (dimArray != NULL) && ok );

    // TODO how to find it???
    //dataAddrMap = rtwCAPI_GetDataAddressMap(mmi);
    //ok = ( (dataAddrMap != NULL) && ok );

    if (ok)
    {
        if (mode == rootInputs)
        {
            nOfSignals = rtwCAPI_GetNumRootInputsFromStaticMap(mmi_address_);
            if (nOfSignals == 0u) {
                RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     No input roots founded");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Input roots founded");
            sigGroup = rootInputs;

        }
        else if (mode == rootOutputs)
        {
            nOfSignals = rtwCAPI_GetNumRootOutputsFromStaticMap(mmi_address_);
            if (nOfSignals == 0u) {
                RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     No Output roots founded");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Output roots founded");
            sigGroup = rootOutputs;

        }
        else
        {
            sigGroup = nullptr;
            RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     Wrong Signal direction");
            return false;
        }
        ok = (sigGroup != NULL);
    }
    RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal Name: %s", sigGroup[0].blockPath ? sigGroup[0].blockPath : "null");
    ok = (sigGroup != NULL);

    return ok;

}




// funzione per il setup dei dati di input e output.
bool SimulinkModelNode::Setup()
{
    RCLCPP_INFO(this->get_logger(), "SETUP FUNCTION: Trying to get the Model Mapping Info data structure from the Simulink shared object...");

    // Simulink instFunction call, dynamic allocation of model data structures
    bool status;
    void *states;
    //data_type_map = nullptr;
    RT_MODEL_libdigitalfilter_T model_instance;
    /*
    datamap_info_function_(&model_instance);
    mmi_ = &(model_instance.DataMapInfo.mmi);
    */
    if (datamap_info_function_ != NULL)
    {
        datamap_info_function_(&model_instance);
        //RCLCPP_INFO(this->get_logger(), "model instance %p", &model_instance);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "SETUP FUNCTION: Failed to initialize the Simulink model");
        return false;
    }
    if (get_mmi_function_ != nullptr)
    {
        //RCLCPP_INFO(this->get_logger(), "funzione mmi%p", get_mmi_function_);

        //void* mmiTemp = (*get_mmi_function_)();
        //RCLCPP_INFO(this->get_logger(), "mmi temp %p", mmiTemp);
        //mmi_ = reinterpret_cast<rtwCAPI_ModelMappingInfo*>(mmiTemp);
        mmi_address_ = (*get_mmi_function_)();
        //RCLCPP_INFO(this->get_logger(), "mmi address: %p", mmi_address_);
    }
    //RCLCPP_INFO(this->get_logger(), "mmi info: %p", mmi_->staticMap);
    status = (mmi_address_ != nullptr);
    if (!status) {
        RCLCPP_ERROR(this->get_logger(), "SETUP FUNCTION: GetMmiPtr function returned a NULL data pointer");
        return false;
    }

    status = (data_type_map != NULL);
    modelNumofInputs =  mmi_address_->Signals.numRootInputs;
    modelNumofOutputs = mmi_address_->Signals.numRootOutputs;
    modelNumofParameters = mmi_address_->Params.numModelParameters;
    RCLCPP_INFO(this->get_logger(), "SETUP FUNCTION: ...obtained.");
    RCLCPP_INFO(this->get_logger(), "-----------------------------");
    RCLCPP_INFO(this->get_logger(), "Number of Model Input: %u",modelNumofInputs );
    RCLCPP_INFO(this->get_logger(), "Number of Model Output: %u",modelNumofOutputs);
    RCLCPP_INFO(this->get_logger(), "Number of Model Parameters: %u",modelNumofParameters);
    RCLCPP_INFO(this->get_logger(), "-----------------------------");
    // -------------------- SETUP COMPLETED (?) ------------------------------------//



    //------------------------------------------------------------------------------//
            // POPULATE MODEL PARAMETERS AND PRINT INFORMATION //
    //------------------------------------------------------------------------------//

    // TODO: finire questa funzione per modelli più complicati.
    status = ScanTunableParameters(mmi_address_);
    if(!status)
    {
        RCLCPP_ERROR(this->get_logger(), "SCAN TUNABLE PARAMETERS FUNCTION NON VALID");
        return false;
    }

    //------------------------------------------------------------------------------//
            // POPULATE MODEL PORTS/SIGNALS AND PRINT INFORMATION //
    //------------------------------------------------------------------------------//

    status = ScanRootIO(mmi_address_, mmi_address_->Signals.rootInputs);
    if(!status)
    {
        RCLCPP_INFO(this->get_logger(), "Scan Root Inputs NON VALID");
        return false;
    }
    status = ScanRootIO(mmi_address_, mmi_address_->Signals.rootOutputs);
    if(!status)
    {
        RCLCPP_INFO(this->get_logger(), "Scan Root Outputs NON VALID");
        return false;
    }



    // SEGMENTATION FAULT

   /*modelNumofInputs = rtwCAPI_GetNumRootInputs(mmi_);
    modelNumofOutputs = rtwCAPI_GetNumRootOutputs(mmi_);
    modelNumofParameters = rtwCAPI_GetNumModelParameters(mmi_);
    RCLCPP_INFO(this->get_logger(), "SETUP FUNTION: Number of Inputs/Outputs/Parameters: %u %u %u", modelNumofInputs, modelNumofOutputs, modelNumofParameters);
    RCLCPP_INFO(this->get_logger(), "SETUP FUNCTION: All right.");
    */

    return true;
}


// è la initialise di MARTe2

/*
    potrebbero essere aggiunti altri parametri come:
     - la verbosità;
     - nonVirtualBusMode;


*/

bool SimulinkModelNode::Initialize(const std::string& library_name, const std::string& symbol_prefix) {

    library_handle_ = dlopen(library_name.c_str(), RTLD_LAZY);

    if (!library_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open library: %s", dlerror());
        return false;
    }

    symbol_prefix_ = symbol_prefix;

    RCLCPP_INFO(this->get_logger(), "Library %s loaded with symbol prefix %s", library_name.c_str(), symbol_prefix_.c_str());

    // caricamento dinamico della libreria tramite simboli

    std::string init_symbol = symbol_prefix_ + "initialize";
    initialize_function_ = (void (*)(void*))dlsym(library_handle_, init_symbol.c_str());
    if (!initialize_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load initialize: %s", dlerror());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "initialize function correctly loaded");
    }


    std::string step_symbol = symbol_prefix_ + "step";
    step_function_ = (void (*)(void*))dlsym(library_handle_, step_symbol.c_str());
    if (!step_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load step: %s", dlerror());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "step function correctly loaded");
    }

    std::string term_symbol = symbol_prefix_ + "terminate";
    terminate_function_ = (void (*)(void*))dlsym(library_handle_, term_symbol.c_str());
    if (!terminate_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load terminate: %s", dlerror());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "terminate function correctly loaded");
    }
    std::string mmi_symbol = symbol_prefix_ + "GetCAPIStaticMap";
    get_mmi_function_ = (const rtwCAPI_ModelMappingStaticInfo* (*)(void))dlsym(library_handle_, mmi_symbol.c_str());
    if (!get_mmi_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load terminate: %s", dlerror());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "getMmifunction correctly loaded");
    }

    std::string datamapinfo_symbol = symbol_prefix_ + "InitializeDataMapInfo";
    datamap_info_function_ = (void (*)(RT_MODEL_libdigitalfilter_T*))dlsym(library_handle_, datamapinfo_symbol.c_str());
    if (!datamap_info_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load terminate: %s", dlerror());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "InitializeDataMapInfo function correctly loaded");
    }





    // Qui puoi chiamare la funzione di inizializzazione della libreria se necessario
    // init_function(...);
    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkModelNode>());
    rclcpp::shutdown();
    return 0;
}
