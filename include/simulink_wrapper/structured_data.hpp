#ifndef SIMULINK_WRAPPER_STRUCTURED_DATA_HPP_
#define SIMULINK_WRAPPER_STRUCTURED_DATA_HPP_

#include <vector>
#include <memory>

class StructuredData {
public:
    StructuredData();

    // Metodo per settare i dati di input
    void SetInputData(const std::vector<double>& input_data);

    // Metodo per ottenere i dati di output
    std::vector<double> GetOutputData() const;

    // Metodo per settare i dati di output (se necessario)
    void SetOutputData(const std::vector<double>& output_data);

    // Metodi per accedere direttamente ai buffer di input e output
    double* GetInputBuffer();
    double* GetOutputBuffer();

private:
    // Buffer di input e output per comunicare con il modello Simulink
    std::vector<double> input_data_;
    std::vector<double> output_data_;

    // Aggiungi altri membri se necessario per la gestione dei dati
};

#endif // SIMULINK_WRAPPER_STRUCTURED_DATA_HPP_
