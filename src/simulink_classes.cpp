#include "simulink_wrapper_cpp/simulink_classes.hpp"


SimulinkDataI::SimulinkDataI()
{

  fullName = "";
  dataClass = "Data";

  numberOfDimensions = 0u;
  for (uint16_T dimIdx = 0u; dimIdx < maxNumOfDims; dimIdx++) {
      numberOfElements[dimIdx] = 1u;
  }
  totalNumberOfElements = 0u;
  orientation = rtwCAPI_SCALAR;

  byteSize     = 0u;
  dataTypeSize = 0u;
  offset       = 0u;

  className     = "void";
  cTypeName     = "void";
  //MARTeTypeName = "void";
  //type = InvalidType;

  address = nullptr;

}

SimulinkSignal::SimulinkSignal() : SimulinkDataI() {

    dataClass = "Signal";

    ROS2Address = nullptr;
    offset       = 0u;
    requiresTransposition = false;
}

void SimulinkDataI::PrintData(const uint64_T maxNameLength /* = 0u */, std::string additionalText /* = "" */)
{
  // Adds spaces at the end of the name until it reaches maxNameLength
  std::string nameWithSpacesAtTheEnd = fullName;
  while (nameWithSpacesAtTheEnd.size() < maxNameLength) {
      nameWithSpacesAtTheEnd += " ";
  }

  RCLCPP_ERROR(rclcpp::get_logger("SimulinkDataI"),
    "%s %s │ dims (%-3u %-3u %-3u) │ elems %-5u │ bytesize %-6u │ %s @%p",
    dataClass.c_str(),
    nameWithSpacesAtTheEnd.c_str(),
    numberOfElements[0u],
    numberOfElements[1u],
    numberOfElements[2u],
    totalNumberOfElements,
    //MARTeTypeName.c_str(),
    byteSize,
    additionalText.c_str(),
    address
  );

}



bool SimulinkDataI::TransposeAndCopy(void *const destination, const void *const source) {
    // Supponendo che il tipo sia sempre Float64MultiArray
    bool ok = TransposeAndCopyT<std_msgs::msg::Float64MultiArray>(destination, source);

    if (!ok) {
        RCLCPP_ERROR(rclcpp::get_logger("SimulinkDataI"), "Error during transpose and copy for Float64MultiArray.");
    }

    return ok;
}

template <typename T>
bool SimulinkDataI::TransposeAndCopyT(void *const destination, const void *const source)
{
    uint32_T numberOfRows    = numberOfElements[0u];
    uint32_T numberOfColumns = numberOfElements[1u];

    for (uint32_T rowIdx = 0u; rowIdx < numberOfRows; rowIdx++) {
        for (uint32_T colIdx = 0u; colIdx < numberOfColumns; colIdx++) {
            // Copia trasposta
            *( (T*) destination + colIdx + numberOfColumns*rowIdx ) = *( (T*) source + rowIdx + numberOfRows*colIdx );
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("SimulinkDataI"), "Transpose and copy completed successfully.");
    return true;
}

void SimulinkSignal::PrintSignal(const uint64_T maxNameLength /* = 0u */ )
{
    SimulinkDataI::PrintData(maxNameLength);
}





/*---------------------------------------------------------------------------*/
/*                               SimulinkPort                                */
/*---------------------------------------------------------------------------*/

SimulinkPort::SimulinkPort() : SimulinkSignal()
{

  isValid      = true;
  isTyped      = false;
  isContiguous = true;
  hasHomogeneousType = true;
  hasHomogeneousOrientation = true;

  runningOffset = 0u;
  typeBasedSize = 0u;
  CAPISize      = 0u;
  offsetBasedSize = 0u;
  baseAddress       = nullptr;
  lastSignalAddress = nullptr;

  const rtwCAPI_ModelMappingStaticInfo* appoggio;
  //TODO---------- is it correct ? -------------//
  mode = appoggio->Signals.rootInputs;
  requiresTransposition = false;

  dataClass = "Port";
  isStructured = true;
}

bool SimulinkPort::AddSignal(SimulinkSignal* const signalIn)
{

  bool ok;
  std::shared_ptr<SimulinkSignal> signalPtr(signalIn);  // Converti SimulinkSignal* in std::shared_ptr<SimulinkSignal>
  carriedSignals.push_back(signalPtr); // Aggiungi a std::list<std::shared_ptr<SimulinkSignal>>
  if (ok)
  {
      uint64_T totalNumOfElems = 1u;
      for (uint32_T elemIdx = 0u; elemIdx < maxNumOfDims; elemIdx++)
      {
          totalNumOfElems *= signalIn->numberOfElements[elemIdx];
      }
      offsetBasedSize = (signalIn->offset) + ( (signalIn->dataTypeSize) * totalNumOfElems );
  }

  return ok;
}


void SimulinkPort::PrintPort(const uint64_T maxNameLength)
{
  std::string typeStr = "";
  if (hasHomogeneousType) {
      typeStr = "homogeneous type";
  }
  else {
      typeStr = "mixed types     ";
  }

  std::string additionalText;
  bool ok;
  try {
    RCLCPP_INFO(
        rclcpp::get_logger("SimulinkPort"),
        "size by type: %-6u │ size by offset %-6u │ %-16s │",
        typeBasedSize, offsetBasedSize, typeStr.c_str()
    );
  } catch (const std::exception& e) {
    ok = false;
    RCLCPP_ERROR(rclcpp::get_logger("SimulinkPort"), "Failed to print additional text: %s", e.what());
  }

  if (!ok) {
      additionalText.clear();
  }

  SimulinkDataI::PrintData(maxNameLength, additionalText);

}

SimulinkInputPort::SimulinkInputPort() : SimulinkPort()
{

  dataClass = "IN  port";
}

SimulinkOutputPort::SimulinkOutputPort() : SimulinkPort()
{

  dataClass = "OUT port";
}


bool SimulinkInputPort::CopyData()
{

  bool ok = true;

  // Copy signal content, telling apart the two modes (struct or byte array)
  // If address==NULL, this signal or port has no corresponding MARTe signal and thus is not mapped
  if (!requiresTransposition)
  {
      if(isStructured)
      {
        for(uint32_T carriedSignalIdx = 0u; (carriedSignalIdx < carriedSignals.size()) && ok; carriedSignalIdx++)
        {
            if(carriedSignals[carriedSignalIdx]->ROS2Address != NULL)
            {
                //std::memcpy(carriedSignals[carriedSignalIdx]->address,carriedSignals[carriedSignalIdx]->ROS2Address,carriedSignals[carriedSignalIdx]->byteSize);
              std::memcpy(carriedSignals[carriedSignalIdx]->address, carriedSignals[carriedSignalIdx]->ROS2Address, carriedSignals[carriedSignalIdx]->byteSize);

            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("SimulinkInputPort"), "Failed to copy memory: null pointer encountered.");
            }
        }
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("SimulinkInputPort"), "Non-structured Port");
        return false;
      }
  }
  else
  {
    if(isStructured)
    {
        for(uint32_T carriedSignalIdx = 0u; (carriedSignalIdx < carriedSignals.size()) && ok; carriedSignalIdx++)
        {
            if(carriedSignals[carriedSignalIdx]->ROS2Address != NULL)
            {
                ok = TransposeAndCopy(carriedSignals[carriedSignalIdx]->address, carriedSignals[carriedSignalIdx]->ROS2Address);
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("SimulinkInputPort"), "Failed to copy memory: null pointer encountered.");
            }
        }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("SimulinkInputPort"), "Non-structured Port");
      return false;
    }
  }

  return ok;
}

bool SimulinkOutputPort::CopyData()
{

  bool ok = true;

  // Copy signal content, telling apart the two modes (struct or byte array)
  // If address==NULL, this signal or port has no corresponding MARTe signal and thus is not mapped
  if (!requiresTransposition)
  {
      if(isStructured)
      {
        for(uint32_T carriedSignalIdx = 0u; (carriedSignalIdx < carriedSignals.size()) && ok; carriedSignalIdx++)
        {
            if(carriedSignals[carriedSignalIdx]->ROS2Address != NULL)
            {
                std::memcpy(carriedSignals[carriedSignalIdx]->ROS2Address, carriedSignals[carriedSignalIdx]->address,carriedSignals[carriedSignalIdx]->byteSize);
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("SimulinkOutputPort"), "Failed to copy memory: null pointer encountered.");
            }
        }
      }
      else
      {
          RCLCPP_ERROR(rclcpp::get_logger("SimulinkOutputPort"), "Non-structured Port");
          return false;
      }
  }
  else
  {
    if(isStructured)
    {
      for(uint32_T carriedSignalIdx = 0u; (carriedSignalIdx < carriedSignals.size()) && ok; carriedSignalIdx++)
      {
          if(carriedSignals[carriedSignalIdx]->ROS2Address != NULL)
          {
              ok = TransposeAndCopy(carriedSignals[carriedSignalIdx]->ROS2Address, carriedSignals[carriedSignalIdx]->address);
          }
      }
    }
    else {
        if(ROS2Address != NULL)
        {
            ok = TransposeAndCopy(ROS2Address, address);
        }
    }
  }
  return ok;
}


