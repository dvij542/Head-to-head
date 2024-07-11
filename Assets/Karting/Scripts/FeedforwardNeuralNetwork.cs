using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.IO;

public class FeedforwardNeuralNetwork : MonoBehaviour
{
    private int[] layers; // Array to hold the number of neurons in each layer
    private int numLayers; // Number of layers in the neural network

    private Matrix<double>[] weights; // Weight matrices for each layer
    private Vector<double>[] biases; // Bias vectors for each layer

    // Constructor to initialize the feedforward neural network
    public FeedforwardNeuralNetwork(int[] layers)
    {
        this.layers = layers;
        numLayers = layers.Length;

        InitializeParameters();
    }

    // Initialize weights and biases for the neural network
    private void InitializeParameters()
    {
        weights = new Matrix<double>[numLayers - 1];
        biases = new Vector<double>[numLayers - 1];

        for (int i = 0; i < numLayers - 1; i++)
        {
            // Initialize weights with random values (you can use other initialization methods)
            weights[i] = Matrix<double>.Build.Random(layers[i + 1], layers[i]);
            // Initialize biases with random values (you can use other initialization methods)
            biases[i] = Vector<double>.Build.Random(layers[i + 1]);
        }
    }

    // ReLU activation function
    private Vector<double> ReLU(Vector<double> x)
    {
        return x.Map(value => Math.Max(0.0, value));
    }

    // Softmax activation function for the output layer
    private Vector<double> Softmax(Vector<double> x)
    {
        double maxVal = x.Maximum();
        Vector<double> expValues = x.Map(value => Math.Exp(value - maxVal));
        double sumExpValues = expValues.Sum();

        return expValues.Map(value => value / sumExpValues);
    }

    // Forward pass to compute the output of the neural network
    public Vector<double> ForwardPass(Vector<double> input)
    {
        Vector<double> output = input;

        for (int i = 0; i < numLayers - 1; i++)
        {
            // Compute the weighted sum of inputs and add the bias
            output = weights[i] * output + biases[i];

            // Apply the activation function (ReLU for hidden layers, Softmax for output layer)
            if (i < numLayers - 2)
                output = ReLU(output);
            else
                output = Softmax(output);
        }

        return output;
    }

    // Backward pass (backpropagation) to update weights and biases during training
    public void BackwardPass(Vector<double> input, Vector<double> targetOutput, double learningRate)
    {
        // First, perform the forward pass to compute intermediate outputs at each layer
        Vector<double>[] layerOutputs = new Vector<double>[numLayers];
        Vector<double> output = input;
        layerOutputs[0] = output;

        for (int i = 0; i < numLayers - 1; i++)
        {
            output = weights[i] * output + biases[i];
            if (i < numLayers - 2)
                output = ReLU(output);
            else
                output = Softmax(output);
            layerOutputs[i + 1] = output;
        }

        // Backpropagate the error and update weights and biases
        Vector<double> error = targetOutput - layerOutputs[numLayers - 1];

        for (int i = numLayers - 2; i >= 0; i--)
        {
            // Compute the derivative of the activation function (ReLU derivative for hidden layers)
            Vector<double> outputDerivative;
            if (i < numLayers - 2)
                outputDerivative = layerOutputs[i + 1].Map(value => value > 0 ? 1.0 : 0.0);
            else
                outputDerivative = layerOutputs[i + 1];

            // Compute the gradient for the weights and biases
            Matrix<double> weightGradient = error.ToColumnMatrix() * outputDerivative.ToRowMatrix() * layerOutputs[i].ToRowMatrix();
            Vector<double> biasGradient = error.PointwiseMultiply(outputDerivative);

            // Update weights and biases using the learning rate
            weights[i] += learningRate * weightGradient;
            biases[i] += learningRate * biasGradient;

            // Propagate the error to the previous layer
            error = weights[i].Transpose() * error;
        }
    }

    // Save the weights and biases to a CSV file
    public void SaveParameters(string filePath)
    {
        using (StreamWriter writer = new StreamWriter(filePath))
        {
            for (int i = 0; i < numLayers - 1; i++)
            {
                // Save the weights for each layer
                for (int row = 0; row < weights[i].RowCount; row++)
                {
                    string rowValues = string.Join(",", weights[i].Row(row).ToArray());
                    writer.WriteLine("weights," + i + "," + row + "," + rowValues);
                }

                // Save the biases for each layer
                string biasValues = string.Join(",", biases[i].ToArray());
                writer.WriteLine("biases," + i + "," + biasValues);
            }
        }
    }

    // Load the weights and biases from a CSV file
    public void LoadParameters(string filePath)
    {
        string[] lines = File.ReadAllLines(filePath);

        for (int i = 0; i < numLayers - 1; i++)
        {
            Matrix<double> weightMatrix = Matrix<double>.Build.Dense(layers[i + 1], layers[i]);
            Vector<double> biasVector = Vector<double>.Build.Dense(layers[i + 1]);

            int weightRowIndex = 0;

            foreach (string line in lines)
            {
                string[] parts = line.Split(',');

                if (parts[0] == "weights" && int.Parse(parts[1]) == i)
                {
                    for (int j = 0; j < weightMatrix.ColumnCount; j++)
                    {
                        double[] rowValues = Array.ConvertAll(parts[3 + j].Split(','), double.Parse);
                        weightMatrix.SetRow(weightRowIndex, rowValues);
                        weightRowIndex++;
                    }
                }

                if (parts[0] == "biases" && int.Parse(parts[1]) == i)
                {
                    double[] biasValues = Array.ConvertAll(parts[2].Split(','), double.Parse);
                    biasVector.SetValues(biasValues);
                }
            }

            weights[i] = weightMatrix;
            biases[i] = biasVector;
        }
    }
}
