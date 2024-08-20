#ifndef SIMULINK_WRAPPER_SIMULINK_CLASSES_HPP_
#define SIMULINK_WRAPPER_SIMULINK_CLASSES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <cstring>
#include "simulink_wrapper_cpp/rtw_capi.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <list>
#include "simulink_wrapper_cpp/rtw_modelmap.h"


struct TypeDescriptor {
    std_msgs::msg::Float64MultiArray type;
    std::string typeName;
    size_t size;
};


class SimulinkDataI {
public:
    std::string dataClass;                 //!< Name of the class (parameter, signal or port).

    std::string fullName;                  //!< Fully expanded Simulink data name.

    uint32_T numberOfDimensions;
    static const uint32_T maxNumOfDims = 3u;             //!< Data number of dimensions
    uint32_T numberOfElements[maxNumOfDims];  //!< Data number of elements in each dimension
    uint32_T totalNumberOfElements;           //!< Total number of elements.

    rtwCAPI_Orientation orientation;        //!< data orientation retrieved from model .so

    uint32_T byteSize;                        //!< Size in bytes occupied by this data.
    uint16_T dataTypeSize;                    //!< Size of the type of this data.
    uint64_T offset;                          //!< Data offset (used if the parameter or signal is part of a structure).

    std::string   className;               //!< data class name (datatype if numeric, class name if struct or enum).
    std::string   cTypeName;               //!< data type name (in C terminology). If this SimulinkDataI is an enumeration this string is not the actual type name and is instead `numeric`.
    //std::string   MARTeTypeName;           //!< data type name (in MARTe terminology). If this SimulinkDataI is an enumeration this string is nevertheless the actual underlying type name.
    TypeDescriptor type;                    //!< data type. If this SimulinkDataI is an enumeration this is nevertheless the actual underlying data type.
    void *address;                          //!< allocated starting address


    /**
     * @brief Default constructor.
     */
    SimulinkDataI();
    /**
     * @brief Destructor.
     */
    /**
     * @brief Prints informations about the element, being it parameter, port or signal.
     * @param[in] maxNameLength max number of characters reserved for the
     *                          parameter name in the printed line.
     */
    void PrintData(const uint64_T maxNameLength = 0u, std::string additionalText = "");
protected:

    /**
     * @brief   Copy data from the source parameter to the model memory
     *          but transpose it in the process.
     * @details This method is used by parameters during actualisation
     *          (see SimulinkParameter::Actualise()) and by signals
     *          (see SimulinkSignal::CopyData()) when they require
     *          transposition. The model almost always uses column-major
     *          orientation to store data, while MARTe2 uses row-major
     *          orientation. Thus, when actualising matrix parameters
     *          or copying matrix signals often transposition is required.
     */
    bool TransposeAndCopy(void *const destination, const void *const source);
    /**
     * @brief Templated version of TransposeAndCopy().
     */
    template<typename T>
    bool TransposeAndCopyT(void *const destination, const void *const source);

};

class SimulinkSignal : public SimulinkDataI
{
public:
    /**
     * @brief Default constructor.
     */
    SimulinkSignal();
    void*  ROS2Address;            //!< Addess of the ROS2 signal that maps this model signal.
    bool requiresTransposition;     //!< `true` if this is signal is a ColumMajor matrix signal (MARTe2 uses row-major data orientation). 
    /**
     * @brief Print information about the signal.
     */
    void PrintSignal(const uint64_T maxNameLength = 0u);
};

class SimulinkPort : public SimulinkSignal
{
public:

    /**
     * @brief Default constructor.
     */
    SimulinkPort();

    std::string invalidationError;     //!< Invalidation error.

    bool isValid;                       //!< `true` if the port is valid.
    bool hasHomogeneousType;            //!< `true` if all signals carried by this port are of the same type.
    bool hasHomogeneousOrientation;     //!< `true` if all signals carried by this port are oriented in the same way.
    bool isTyped;                       //!< `true` if #type and #orientation for this port has already been set.
    bool isContiguous;                  //!< `true` if the port data is contiguous.
    //TODO Verify
    bool isStructured;                   //!< `true` if the port has to be treated like a structure of its child signals

    uint32_T runningOffset;
    uint64_T typeBasedSize;
    uint64_T offsetBasedSize;
    uint32_T CAPISize;

    void*  baseAddress;
    void*  lastSignalAddress;

    /**
     * @brief   Direction of the signal.
     * @details Can be input (`mode = InputSignals`) or output (`mode = OutputSignals`).
     */

    const rtwCAPI_Signals* mode;


    /**
     * @brief A list of all signals passing through this port.
     */
    std::vector<std::shared_ptr<SimulinkSignal>> carriedSignals;
    /**
     * @brief   Adds a signal to the list of signals carried by this port.
     * @details The method also increment the #offsetBasedSize of this port
     *          based on the offset of the new signal.
     * @return  `true` if the signal was correctly added to the list.
     */
    bool AddSignal(SimulinkSignal* const signalIn);

    /**
     * @brief  Check the contiguity of the port data.
     * @return `true` if the port has contiguous data.
     */
    //inline bool IsContiguous() const { return (CAPISize == offsetBasedSize); }

    /**
     * @brief Print information about this port.
     */
    void PrintPort(const uint64_T maxNameLength);

    /**
     * @brief Copy data from the associated ROS2 signal to the associated model port.
     */
    virtual bool CopyData(void) = 0;
};


/*---------------------------------------------------------------------------*/
/*                             SimulinkInputPort                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief   Class that manages informations about a Simulink(R) model input port retirieved
 *          from the model shared library.
 */
class SimulinkInputPort : public SimulinkPort {
public:

  /**
   * @brief Default constructor.
   */
  SimulinkInputPort();

  /**
   * @brief   Copy data from the associated MARTe2 signal to the associated model port.
   * @details The method checks if the signal needs transposition
   *          before copying, and transpose the signal if needed.
   *          Only column-major matrix signals require transposition.
   *          MARTe2 works in row-major orientation, so if the model
   *          has column-major matrix signals they get transposed.
   * @returns `true` if data is successfully copied, `false` otherwise.
   */
  virtual bool CopyData();
};

/*---------------------------------------------------------------------------*/
/*                            SimulinkOutputPort                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief   Class that manages informations about a Simulink(R) model output port retirieved
 *          from the model shared library.
 */
class SimulinkOutputPort : public SimulinkPort {
public:

  /**
   * @brief Default constructor.
   */
  SimulinkOutputPort();
  /**
   * @brief Copy data from the associated model port to the associated MARTe2 signal.
   * @details The method checks if the signal needs transposition
   *          before copying, and transpose the signal if needed.
   *          Only column-major matrix signals require transposition.
   *          MARTe2 works in row-major orientation, so if the model
   *          has column-major matrix signals they get transposed.
   * @returns `true` if data is successfully copied, `false` otherwise.
   */
  virtual bool CopyData();
};








































#endif // SIMULINK_WRAPPER_SIMULINK_CLASSES_HPP_