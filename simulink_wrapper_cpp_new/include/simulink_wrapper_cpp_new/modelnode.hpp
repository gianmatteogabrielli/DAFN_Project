#ifndef SIMULINK_WRAPPER_MODELNODE_HPP_
#define SIMULINK_WRAPPER_MODELNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <dlfcn.h>
#include "simulink_wrapper_cpp_new/rtw_modelmap.h"
#include <vector>



struct SignalDimensions {
    uint32_t dimension;     // dimensione del segnale: scalare, bidimensionale, etc...
    uint32_t arrayPosition;
    std::vector<int> arrayDimensions;   //rows, cols, layers
    int vardim;         // 0 se non varia, 1 else

};

class SimulinkModelNode : public rclcpp::Node {
public:
    SimulinkModelNode();

private:
    // ------------------ PARAM SECTION ---------------------//
    void* states;
    std::string simulinkLibraryName;
    std::string symbol_prefix_;
    void* library_handle_;
    const rtwCAPI_ModelParameters* modelParams;     //!< Pointer to the structure storing parameter data.
    const rtwCAPI_Signals*         rootInputs;      //!< Pointer to the structure storing root input data.
    const rtwCAPI_Signals*         rootOutputs;     //!< Pointer to the structure storing root output data.
    const rtwCAPI_Signals*         sigGroup;        //!< Pointer to rootInputs or rootOutputs depending on what structure the GAM is parsing.
    const rtwCAPI_DataTypeMap*     dataTypeMap;     //!< Pointer to the structure storing data types of parameters, inputs and outputs.
    const rtwCAPI_ElementMap*      elementMap;      //!< Pointer to the structure storing data for elements of a structured parameter or signal.
    const rtwCAPI_DimensionMap*    dimMap;          //!< Pointer to the structure storing the dimensions of parameters, inputs and outputs.
    const uint32_T*                  dimArray;        //!< Pointer to the array storing the number of elements for each dimension of parameters, inputs and outputs.
    void**                         dataAddrMap;     //!< Pointer to the structure storing the memory location of parameters, inputs and outputs.
    void* inputAddress;
    void* outputAddress;
    uint16_T inputDimMapIndex;
    uint16_T outputDimMapIndex;
    SignalDimensions inputDimension;
    SignalDimensions outputDimension;
    uint_T NumOfExpElem;


    uint32_T modelNumOfInputs;            //!< Number of inputs of the model.
    uint32_T modelNumOfOutputs;           //!< Number of outputs of the model.
    uint32_T modelNumOfParameters;        //!< Number of tunable parameters of the model.

    // ------------------ VARIABLE PARAM SECTION ---------------------//
    bool verbose;
    rcl_interfaces::msg::ParameterDescriptor param_descriptor_;

    OnSetParametersCallbackHandle::SharedPtr param_clbk_handle_;
    rcl_interfaces::msg::SetParametersResult param_clbk(const std::vector<rclcpp::Parameter> &params);


    // ------------------ FUNCTION SECTION ---------------------//
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    bool Initialize(const std::string& library_name, const std::string& symbol_prefix);
    bool Setup();
    bool ScanRootIO(rtwCAPI_ModelMappingInfo* mmi, const rtwCAPI_Signals* mode);
    bool verifyDimensions(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void print_matrix(const std_msgs::msg::Float64MultiArray msg);
    void* (*inst_function_)(void);
    void (*init_function_)(void *);
    void (*step_function_)(void*);
    void* (*get_mmi_function_)(void*);
    void (*terminate_function_)(void *);

    // ------------------ NODE SECTION ---------------------//
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_node;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_node;

};

















#endif // SIMULINK_WRAPPER_MODELNODE_HPP_
