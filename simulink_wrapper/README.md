# SIMULINK WRAPPER FOR ROS2

## Project Idea

Implement a ROS2 architecture consisting of

1. A first node, defined as publisher, that publishes input data for the Simulink model on a specific topic.
2. An intermediate node, the actual Simulink wrapper node, which will read the model data from the input topic and return the output data by writing them to a second topic.
3. A final node, defined as a subscriber, which will read the results of the Simulink model from the second topic.

## Simulink Wrapper Node

The first step is to transform any Simulink model into a dynamic library (e.g., "name_project.so"). This can be achieved using software provided by MathWorks, such as:

- Embedded Coder
- Simulink Coder

However, they present a limitation: they only provide a static mapping of the model, not a dynamic one. This was confirmed by inspecting the symbols defined inside the "name_project.so" file with the following command:

`nm <name_project.so> | less`

For this reason, a different approach was taken, which involves implementing a MATLAB code that allows the model to return the desired dynamic mapping.

```md
model_name = “model_name”;
% Solver
set_param(model_name, 'SolverType', 'Fixed-step');
% Code Generation
set_param(model_name, 'SystemTargetFile', 'ert_shrlib.tlc');
set_param(model_name, 'RTWVerbose',
0
);
% Optimization
set_param(model_name, 'DefaultParameterBehavior', 'Tunable');
set_param(model_name, 'OptimizationCustomize',
1
);
set_param(model_name, 'GlobalVariableUsage',
'None'
);
% Report
set_param(model_name, 'GenerateReport', 0);
% Comments
set_param(model_name, 'GenerateComments', 0);
% Custom code (MODEL is a coder varialbe for the model name)
set_param(model_name, 'CustomSourceCode', ...
[ ...
'#define CONCAT(str1, str2, str3) CONCAT_(str1, str2, str3)'
'#define CONCAT_(str1, str2, str3) str1 ## str2 ## str3'
'#define GET_MMI_FUNC
CONCAT(MODEL
, _GetCAPImmi ,
)'
'#define RT_MODEL_STRUCT CONCAT(RT_MODEL_ , MODEL
, _T )'
newline, ...
newline, ...
newline, ...
newline, ...
newline, ...
'void* GET_MMI_FUNC(void* voidPtrToRealTimeStructure)'
newline, ...
'{'
newline, ...
'
rtwCAPI_ModelMappingInfo* mmiPtr = &(rtmGetDataMapInfo( ( RT_MODEL_STRUCT *
)(voidPtrToRealTimeStructure) ).mmi);' newline, ...
'
return (void*) mmiPtr;'
newline, ...
'}' ...
] ...
);
% Interface
set_param(model_name, 'SupportComplex',
0);
set_param(model_name, 'SupportAbsoluteTime', 0);
set_param(model_name, 'SuppressErrorStatus', 1);
set_param(model_name, 'CodeInterfacePackaging', 'Reusable function');
set_param(model_name, 'RootIOFormat', 'Part of model data structure');
set_param(model_name, 'RTWCAPIParams', 1);
set_param(model_name, 'RTWCAPIRootIO', 1);
set_param(model_name, 'GenerateAllocFcn', 1);
set_param(model_name, 'IncludeMdlTerminateFcn',
0);
set_param(model_name, 'CombineSignalStateStructs', 1);
set_param(model_name, 'ArrayLayout', 'Column-major');
% Templates
set_param(model_name, 'GenerateSampleERTMain', 0);
```

**HOW TO USE IT?**

1. Enter in the folder where the simulink model is defined.
2. Save the matlab script defined previously in that folder.
3. Open the simulink model.
4. Rename all the signal blocks with names that you prefer (this is very important!)
5. Open Embedded Coder, (Avalaible in "Add-Ons").
6. Run the matlab script.
7. In the Embedded Coder interface, press "Build".

If you followed this instruction, there will be a folder with all C/C++ file which describe your model and a file named <name_project.so>.

Once the .so file is obtained, it must be placed in the include folder of the ROS2 project. In addition to the typical header files for each node, five additional libraries from MATLAB and Simulink must be included:

1. rtw_capi.h
2. rtw_modelmap.h
3. rtw_modelmap_logging.h
4. rtw_types.h (found in the folder containing the .c/.cpp files of the project)
5. sysran_types.h

Now, one can proceed to load dynamically the file.so in the ROS2 project.
By using the dynamic symbol loading mechanism, you can load and manage symbols from the shared library (the .so file) at runtime. In a typical ROS2 project, this can be done using the dlopen and dlsym functions from the standard POSIX library.

(see Initialize() function, in modelnode.cpp, 423 - 502)
(note that it is necessary to write the absolut path of the file.so in the prefix: libraryName.

    This is defined as a parameter of the ROS2 model node in the config file  (.yaml)).

**ALL THE FUNCTIONS THAT HAVE BEEN LOADED ARE DESCRIBED IN THE HEADER FILE modelnode.hpp**

## Signal Structure

One of the most important thing is to understand how signals are stuctured by simulink.
In this project, they are described by

```md
const rtwCAPI_ModelParameters* modelParams;     //!< Pointer to the structure storing parameter data.
const rtwCAPI_Signals*         rootInputs;      //!< Pointer to the structure storing root input data.
const rtwCAPI_Signals*         rootOutputs;     //!< Pointer to the structure storing root output data.
const rtwCAPI_Signals*         sigGroup;        //!< Pointer to rootInputs or rootOutputs depending on what structure the node is parsing.
const rtwCAPI_DataTypeMap*     dataTypeMap;     //!< Pointer to the structure storing data types of parameters, inputs and outputs.
const rtwCAPI_ElementMap*      elementMap;      //!< Pointer to the structure storing data for elements of a structured parameter or signal.
const rtwCAPI_DimensionMap*    dimMap;          //!< Pointer to the structure storing the dimensions of parameters, inputs and outputs.
const uint32_T*                  dimArray;        //!< Pointer to the array storing the number of elements for each dimension of parameters, inputs and outputs.
uint32_T dimIndex;
void**                         dataAddrMap;     //!< Pointer to the structure storing the memory location of parameters, inputs and outputs.
```

One of the most important parameter is "dimIndex".

This field indicates the index in the dimension map (dimensionMap) that specifies the dimensions of the signal. For example, in the case of a scalar, we have that

```md
static rtwCAPI_DimensionMap rtDimensionMap[] = {
    /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */

  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 2, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 4, 2, 0 }
};

```


The number of field in rtDimensionMap is equal to the number of signal inputs and outputs of the model.

**REMARK**

**If one is operating with a model defined by an input and output signals with the same dimensions, in rtDimensionMap, it will appear only one field!**

In this case, index value will be "1".

- rtwCAPI_SCALAR: Indicates that the signal is a scalar, meaning it does not have dimensions greater than 1x1.
- 0: This is the index in the dimensionArray from which the dimensions for this signal start.
- 2: Indicates the number of dimensions. Even though the signal is scalar, this field can often be 2 (for example, 1x1 for a scalar).
- 0: This field indicates the index in the data map (if applicable), but for scalar signals, it is often 0. Not used in this project.

The second value of the field {...} in the dimension map will return the index of the dimension array, which contains the actual dimensions of the signal.

```md
static uint_T rtDimensionArray[] = {
  1,
  1,
  4,
  5,
  5,
  4
};
```

All the signal dimensions taken from the simulink model are saved in a struct, defined in modelnode.hpp. See below

```md
struct SignalDimensions {
    uint32_t dimension;     // Signal dimensions: scalar signal, bidimensional signal (e.g matrix), etc...
    uint32_t arrayPosition;
    std::vector<int> arrayDimensions;   //arrayDimensions[0] = nrows, arrayDimensions[1] = ncolumns, arrayDimensions[2] = nlayers
    int vardim;         // 0 if not change, 1 else

};
```

Each pair (or triple) of values in the rtDimensionArray represents the actual dimension of a signal, as specified by the indices in the rtDimensionMap.
The code for accessing information about the dimensions of input and output signals can be found between lines 279 and 312 of the ScanRootIO function.

