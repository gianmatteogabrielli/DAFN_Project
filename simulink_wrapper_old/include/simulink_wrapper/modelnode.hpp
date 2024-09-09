#ifndef SIMULINK_WRAPPER_MODELNODE_HPP_
#define SIMULINK_WRAPPER_MODELNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <dlfcn.h>
#include "simulink_wrapper_cpp/rtw_capi.h"
#include "simulink_wrapper_cpp/rtw_modelmap.h"
#include "simulink_wrapper_cpp/libdigitalfilter.h"
#include "simulink_wrapper_cpp/libdigitalfilter_capi.h"
#include "simulink_wrapper_cpp/simulink_classes.hpp"

class SimulinkModelNode : public rclcpp::Node {
public:
    SimulinkModelNode();

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    bool Initialize(const std::string& library_name, const std::string& symbol_prefix);
    bool Setup();
    bool ScanTunableParameters(const rtwCAPI_ModelMappingStaticInfo* mmiAddress);
    bool ScanRootIO(const rtwCAPI_ModelMappingStaticInfo* mmiAddress, const rtwCAPI_Signals* mode);
    std::string simulinkLibraryName;
    std::string symbol_prefix_;
    std::vector<std::shared_ptr<void>> lastAddressVector;
    SimulinkPort* currentPort;
    std::string signalSeparator;
    void* library_handle_;

    void (*initialize_function_)(void *);
    void (*step_function_)(void *);
    void (*terminate_function_)(void *);
    const rtwCAPI_ModelMappingStaticInfo* (*get_mmi_function_)(void);
    void (*datamap_info_function_)(RT_MODEL_libdigitalfilter_T*);   // instFunction of MARTe2
    const rtwCAPI_ModelMappingStaticInfo* mmi_address_;
    const rtwCAPI_DataTypeMap* data_type_map;
    uint_T modelNumofInputs;
    uint_T modelNumofOutputs;
    uint_T modelNumofParameters;


    const rtwCAPI_ModelParameters* modelParams;     //!< Pointer to the structure storing parameter data.
    const rtwCAPI_Signals*         rootInputs;      //!< Pointer to the structure storing root input data.
    const rtwCAPI_Signals*         rootOutputs;     //!< Pointer to the structure storing root output data.
    const rtwCAPI_Signals*         sigGroup;        //!< Pointer to rootInputs or rootOutputs depending on what structure the GAM is parsing.
    const rtwCAPI_DataTypeMap*     dataTypeMap;     //!< Pointer to the structure storing data types of parameters, inputs and outputs.
    const rtwCAPI_ElementMap*      elementMap;      //!< Pointer to the structure storing data for elements of a structured parameter or signal.
    const rtwCAPI_DimensionMap*    dimMap;          //!< Pointer to the structure storing the dimensions of parameters, inputs and outputs.
    const uint32_T*                  dimArray;        //!< Pointer to the array storing the number of elements for each dimension of parameters, inputs and outputs.
    void**                         dataAddrMap;     //!< Pointer to the structure storing the memory location of parameters, inputs and outputs.

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_node;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_node;

};

#endif // SIMULINK_WRAPPER_MODELNODE_HPP_
