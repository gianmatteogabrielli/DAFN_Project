#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper_cpp_new/modelnode.hpp"


SimulinkModelNode::SimulinkModelNode()
: Node("simulink_modelNode"){

    RCLCPP_INFO(this->get_logger(), "SIMULINK NODE (select verbosity: true to see logs)");
    // Inizializza la libreria Simulink
    simulinkLibraryName = "/home/neo/workspace/src/cpp/simulink_wrapper_cpp_new/mylibs/libdigitalfilter.so";
    symbol_prefix_ = "libdigitalfilter";


    // Register parameter set callback
    param_clbk_handle_ = this->add_on_set_parameters_callback(
    std::bind(
      &SimulinkModelNode::param_clbk,
      this,
      std::placeholders::_1));

    // Create descriptor for the boolean parameter
    param_descriptor_.set__name("verbose");
    param_descriptor_.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
    param_descriptor_.set__description("Set the code verbose.");
    param_descriptor_.set__additional_constraints(
    "The parameter accepts only boolean value.");
    param_descriptor_.set__read_only(false);
    param_descriptor_.set__dynamic_typing(false);

    // Declare the boolean parameter with a default value of false
    this->declare_parameter("verbose", true, param_descriptor_);





    if (!Initialize(simulinkLibraryName, symbol_prefix_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize the Simulink model");
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "The Simulink Model is been initialized.\n");
    }
    if(!Setup())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup the Simulink model");
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "Setup the Simulink model has been done succesfully");
    }

    subscriber_node = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "simulink_input", 10, std::bind(&SimulinkModelNode::topic_callback, this, std::placeholders::_1));

    publisher_node = this->create_publisher<std_msgs::msg::Float64MultiArray>("simulink_output", 10);

}

bool SimulinkModelNode::verifyDimensions(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if(verbose) RCLCPP_INFO(this->get_logger(), "In verifyDimension.");

    NumOfExpElem = 1;

    for(uint32_t i = 0; i < inputDimension.dimension; i++)
    {
        if(msg->layout.dim[i].size != inputDimension.arrayDimensions[i])
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nelle dimensioni del segnale di ingresso.");
            return false;
        }

        NumOfExpElem = NumOfExpElem *inputDimension.arrayDimensions[i];
    }

    if(msg->data.size() != NumOfExpElem)
    {
        RCLCPP_ERROR(this->get_logger(), "Errore nelle dimensioni del segnale di ingresso.");
        return false;
    }

    return true;
}




rcl_interfaces::msg::SetParametersResult SimulinkModelNode::param_clbk(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  rcl_interfaces::msg::SetParametersResult res{};
  res.set__successful(false);
  res.set__reason("Invalid parameters");

  // Look for valid parameters to update, and populate result accordingly
  for (const rclcpp::Parameter & p : params) {
    if ((p.get_name() == "verbose") && (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Requested parameter change to: %s \n",
        p.as_bool() ? "true" : "false");
      bool new_val = p.as_bool();

      // Updating the class member with the new value
      verbose = new_val;

      res.set__successful(true);
      res.set__reason("");
      break;
    }
  }

  return res;
}






void SimulinkModelNode::topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

        if(!verifyDimensions(msg))
        {
            return;
        }
        std::memcpy(inputAddress, msg->data.data(), msg->data.size() * sizeof(double));
        if(step_function_ != NULL)
        {
            (*step_function_)(states);
        }
        double* output_double_ptr = static_cast<double*>(outputAddress);
        if(verbose) print_matrix(*msg);
        // Stampa l'output per il debugging
        if(verbose) RCLCPP_INFO(this->get_logger(), "Output from Simulink model: %f %f %f %f",
            output_double_ptr[0], output_double_ptr[1], output_double_ptr[2], output_double_ptr[3]);

        // Prepara il messaggio da pubblicare
        auto output_message = std_msgs::msg::Float64MultiArray();

        // Aggiungi ogni elemento del risultato Simulink al vettore data
        output_message.layout.dim.resize(outputDimension.dimension);
        for (size_t i = 0; i < NumOfExpElem; ++i){
            output_message.data.push_back(output_double_ptr[i]);
        }
        for(size_t i = 0; i < outputDimension.dimension; ++i)
        {
            output_message.layout.dim[i].size = outputDimension.arrayDimensions[i];
        }
        // Pubblica l'output
        if(verbose) print_matrix(output_message);
        publisher_node->publish(output_message);

}


bool SimulinkModelNode::ScanRootIO(rtwCAPI_ModelMappingInfo* mmi, const rtwCAPI_Signals* mode)
{
    uint32_T       nOfSignals = 0u;
    std::string sigName;
    uint16_T       dataTypeIdx;
    //uint8_T       slDataID;
    //uint16_T      numElements;
    //uint16_T       dataTypeSize;
    uint32_T       addrIdx;
    void*        sigAddress;

    bool ok = (mmi != NULL);
    // Populating C API data structure pointers of the class from mmi
    rootInputs = rtwCAPI_GetRootInputs(mmi);
    ok = ( (rootInputs != NULL) && ok );

    rootOutputs = rtwCAPI_GetRootOutputs(mmi);
    ok = ( (rootOutputs != NULL) && ok );

    dataTypeMap = rtwCAPI_GetDataTypeMap(mmi);
    ok = ( (dataTypeMap != NULL) && ok );

    elementMap = rtwCAPI_GetElementMap(mmi);
    ok = ( (elementMap != NULL) && ok );

    dimMap = rtwCAPI_GetDimensionMap(mmi);
    ok = ( (dimMap != NULL) && ok );

    dimArray = rtwCAPI_GetDimensionArray(mmi);
    ok = ( (dimArray != NULL) && ok );

    dataAddrMap = rtwCAPI_GetDataAddressMap(mmi);
    ok = ( (dataAddrMap != NULL) && ok );
    if (ok)
    {
        if (mode == rootInputs)
        {
            nOfSignals = rtwCAPI_GetNumRootInputs(mmi);
            if (nOfSignals == 0u) {
                RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     No input roots founded");
                return false;
            }
            if(verbose) RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Input roots founded");
            sigGroup = rootInputs;
        }
        else if (mode == rootOutputs)
        {
            nOfSignals = rtwCAPI_GetNumRootOutputs(mmi);
            if (nOfSignals == 0u) {
                RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     No Output roots founded");
                return false;
            }
            if(verbose) RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Output roots founded");
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

    if(verbose) RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal Name: %s", sigGroup[0].blockPath ? sigGroup[0].blockPath : "null");

    ok = (sigGroup != NULL);
    for (uint16_T sigIdx = 0u; (sigIdx < nOfSignals) && ok ; sigIdx++)
    {
        dataTypeIdx  = rtwCAPI_GetSignalDataTypeIdx(sigGroup, sigIdx);
        sigName      = rtwCAPI_GetSignalName(sigGroup, sigIdx);
        //slDataID     = rtwCAPI_GetDataTypeSLId(dataTypeMap, dataTypeIdx);
        //numElements  = rtwCAPI_GetDataTypeNumElements(dataTypeMap,dataTypeIdx);
        //dataTypeSize = rtwCAPI_GetDataTypeSize(dataTypeMap,dataTypeIdx);

        addrIdx = rtwCAPI_GetSignalAddrIdx(sigGroup,sigIdx);
        sigAddress = (void *) rtwCAPI_GetDataAddress(dataAddrMap,addrIdx);
        //RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %u address: %p", sigGroup[0].sysNum, sigAddress);
        if(mode == rootInputs)
        {
            inputAddress = sigAddress;
            if(verbose)
            {
                RCLCPP_INFO(this->get_logger(), "//------------------------- INPUT ADDRESS ----------------------//");
                RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %s address: %p",  sigGroup[sigIdx].blockPath, inputAddress);
                RCLCPP_INFO(this->get_logger(), "//--------------------------------------------------------------//");
                RCLCPP_INFO(this->get_logger(), "//-------------------------- INPUT SIZE ----------------------//");

            }
            uint_T dimIndex = mmi->staticMap->Signals.rootInputs->dimIndex;
            inputDimension.arrayPosition = dimMap[dimIndex].dimArrayIndex;
            inputDimension.dimension = dimMap[dimIndex].numDims;   //1 scalar, 2 matrix or vector, 3 3-D matrix
            for(uint32_t i = 0; i < inputDimension.dimension; i++ )
            {
                inputDimension.arrayDimensions.push_back(mmi->staticMap->Maps.dimensionArray[inputDimension.arrayPosition + i]);
            }
            //inputDimension.numRows = mmi->staticMap->Maps.dimensionArray[inputDimension.arrayPosition];
            //inputDimension.numCols = mmi->staticMap->Maps.dimensionArray[inputDimension.arrayPosition + 1];
            if(verbose)
            {
                RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %s dimension: %u %u",  sigGroup[sigIdx].blockPath, inputDimension.arrayDimensions[0], inputDimension.arrayDimensions[1]);
                RCLCPP_INFO(this->get_logger(), "//--------------------------------------------------------------// \n");
            }
        }
        else
        {
            outputAddress = sigAddress;
            if(verbose)
            {
                RCLCPP_INFO(this->get_logger(), "//------------------------- OUTPUT ADDRESS ----------------------//");
                RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %s address: %p",  sigGroup[sigIdx].blockPath, outputAddress);
                RCLCPP_INFO(this->get_logger(), "//--------------------------------------------------------------//");
                RCLCPP_INFO(this->get_logger(), "//------------------------- OUTPUT SIZE ----------------------//");

            }
            uint_T dimIndex = mmi->staticMap->Signals.rootOutputs->dimIndex;
            outputDimension.arrayPosition = dimMap[dimIndex].dimArrayIndex;
            outputDimension.dimension = dimMap[dimIndex].numDims;   //1 scalar, 2 matrix or vector, 3 3-D matrix
            for(uint32_t i = 0; i < outputDimension.dimension; i++ )
            {
                outputDimension.arrayDimensions.push_back(mmi->staticMap->Maps.dimensionArray[outputDimension.arrayPosition + i]);
            }
            //outputDimension.numRows = mmi->staticMap->Maps.dimensionArray[outputDimension.arrayPosition];
            //outputDimension.numCols = mmi->staticMap->Maps.dimensionArray[outputDimension.arrayPosition + 1];
            if(verbose)
            {
                RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %s dimension: %u %u",  sigGroup[sigIdx].blockPath, outputDimension.arrayDimensions[0], outputDimension.arrayDimensions[1]);
                RCLCPP_INFO(this->get_logger(), "//--------------------------------------------------------------//\n");

            }
            //RCLCPP_INFO(this->get_logger(), "SCAN ROOT IO FUNCTION      Signal %s dimension: %u %u",  sigGroup[sigIdx].blockPath, outputDimension.numRows, outputDimension.numCols);
        }


    }
    if(!ok)
    {
        RCLCPP_ERROR(this->get_logger(), "SCAN ROOT IO FUNCTION     Error in sigGroup");
        return false;

    }

    return ok;

}




bool SimulinkModelNode::Setup()
{
    if(verbose) RCLCPP_INFO(this->get_logger(), "SETUP FUNCTION: Trying to get the Model Mapping Info data structure from the Simulink shared object...");

    // Simulink instFunction call, dynamic allocation of model data structures
    bool status;

    if (inst_function_ != nullptr)
    {
        states = (*inst_function_)();
        if(verbose) RCLCPP_INFO(this->get_logger(), "Chiamata a inst_function()\n");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "SETUP FUNCTION: Simulink model allocation function returned a NULL data pointer");
        return false;
    }
    rtwCAPI_ModelMappingInfo* mmi = (rtwCAPI_ModelMappingInfo*)nullptr;
    if(get_mmi_function_ != nullptr)
    {
        mmi = (rtwCAPI_ModelMappingInfo*)get_mmi_function_(states);
    }
    if(mmi != nullptr)
    {
        dataTypeMap = rtwCAPI_GetDataTypeMap(mmi);
        modelNumOfInputs     = rtwCAPI_GetNumRootInputs(mmi);
        modelNumOfOutputs    = rtwCAPI_GetNumRootOutputs(mmi);
        modelNumOfParameters = rtwCAPI_GetNumModelParameters(mmi);
    }
    /*
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
    */

    status = ScanRootIO(mmi, mmi->staticMap->Signals.rootInputs);
    if(!status)
    {
        RCLCPP_ERROR(this->get_logger(), "Scan Root Inputs NON VALID");
        return false;
    }
    status = ScanRootIO(mmi, mmi->staticMap->Signals.rootOutputs);
    if(!status)
    {
        RCLCPP_ERROR(this->get_logger(), "Scan Root Outputs NON VALID");
        return false;
    }
    else
    {
         // deve stare in SETUP
        if(verbose)
        {
            RCLCPP_INFO(this->get_logger(), "SETUP FUNTION: Number of Inputs/Outputs/Parameters: %u %u %u", modelNumOfInputs, modelNumOfOutputs, modelNumOfParameters);
            RCLCPP_INFO(this->get_logger(), "SETUP FUNCTION      Init of the simulink model...");

        }

        (*init_function_)(states);

    }


    return true;
}




bool SimulinkModelNode::Initialize(const std::string& library_name, const std::string& symbol_prefix) {

    RCLCPP_INFO(this->get_logger(), "INITIALIZE FUNCTION: loaded dependencies on ROS2_node");

    library_handle_ = dlopen(library_name.c_str(), RTLD_LAZY);

    if (!library_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open library: %s", dlerror());
        return false;
    }

    symbol_prefix_ = symbol_prefix;

    if(verbose) RCLCPP_INFO(this->get_logger(), "Library %s loaded with symbol prefix %s", library_name.c_str(), symbol_prefix_.c_str());

    // caricamento dinamico della libreria tramite simboli
    inst_function_ = (void* (*)())dlsym(library_handle_, symbol_prefix_.c_str());
    if(!inst_function_ )
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't find %s symbol in model library (inst_function == NULL).", symbol_prefix_.c_str());
        return false;
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "inst_function_() correctly loaded.");
    }


    std::string init_symbol = symbol_prefix_ + "_initialize";
    init_function_ = (void (*)(void*))dlsym(library_handle_, init_symbol.c_str());
    if (!init_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load initialize: %s", dlerror());
        return false;
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "init_function_() correctly loaded.");
    }


    std::string step_symbol = symbol_prefix_ + "_step";
    step_function_ = (void (*)(void*))dlsym(library_handle_, step_symbol.c_str());
    if (!step_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load step: %s", dlerror());
        return false;
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "step_function_() correctly loaded.");
    }

    std::string mmi_symbol = symbol_prefix_ + "_GetCAPImmi";
    get_mmi_function_ = (void* (*)(void*))dlsym(library_handle_, mmi_symbol.c_str());
    if (!get_mmi_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load terminate: %s", dlerror());
        return false;
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "get_mmi_function_() correctly loaded.");
    }


    std::string terminate_symbol = symbol_prefix_ + "_terminate";
    terminate_function_ = (void (*)(void*))dlsym(library_handle_, terminate_symbol.c_str());
    if (!terminate_function_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load terminate_function_: %s", dlerror());
        return false;
    }
    else
    {
        if(verbose) RCLCPP_INFO(this->get_logger(), "terminate_function_() correctly loaded.");
    }


    // Qui puoi chiamare la funzione di inizializzazione della libreria se necessario
    // init_function(...);
    return true;
}

void SimulinkModelNode::print_matrix(const std_msgs::msg::Float64MultiArray msg) {


    size_t rows = msg.layout.dim[0].size;
    size_t cols = msg.layout.dim[1].size;

    std::stringstream ss;
    if (msg.data.size() != rows * cols) {
        RCLCPP_ERROR(this->get_logger(), "Data size does not match the dimensions specified in the layout.");
        return;
    }


    // Stampa la matrice riga per riga
    for (size_t i = 0; i < rows; ++i) {
        ss << "[ ";
        for (size_t j = 0; j < cols; ++j) {
            ss << msg.data[i * cols + j] << " ";
        }
        ss << "]\n";
    }

    // Usa RCLCPP_INFO per stampare la matrice
    RCLCPP_INFO(this->get_logger(), "\n%s", ss.str().c_str());
}












int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkModelNode>());
    rclcpp::shutdown();
    return 0;
}
