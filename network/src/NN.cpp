#include "NN.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

NN::NN(const string net_path) {
	FILE *fp = fopen(net_path.c_str(), "r");
	if (!fp) {
		puts("Read neural network weight error!");
		exit(-1);
	}
    if (fscanf(fp, "%d", &n_layers) == EOF) {
        puts("Read neural network weight error1!");
        exit(-1);
    }
	for (int i = 0; i < n_layers; ++i) {
        if (fscanf(fp, "%d", &n_layerNodes[i]) == EOF) {
            puts("Read neural network weight error2!");
            exit(-1);
        }
	}

	int n_weight;
    weight.clear();
    int cnt = 0;
	for (int i = 1; i < n_layers; ++i) {
		n_weight = n_layerNodes[i] * (n_layerNodes[i-1] + 1);
        vector<float> vt(n_weight, 0);
		for (int j = 0; j < n_weight; ++j) {
            if (fscanf(fp, "%f", &vt[j]) == EOF) {
                puts("Read neural network weight error3!");
                exit(-1);
            }
		}
		weight.push_back(vt);
	}
    //printf("finish\n");
	fclose(fp);
}

float NN::sigmoid(float z) {
	return 1.0 / (1 + exp(-z));
}

vector<float> NN::predict(vector<float> input) {
    vector<float> output;
    float z;
	for (int i = 1; i < n_layers; ++i) {
		for (int j = 0, src = 0; j < n_layerNodes[i]; ++j, src += n_layerNodes[i-1] + 1) {
			z = weight[i-1][src];
			for (int k = 0; k < n_layerNodes[i-1]; ++k) {
				z += weight[i-1][src + k + 1] * input[k];
			}
			output.push_back(sigmoid(z));
		}
		if (i + 1 == n_layers) break;
		input = output;
		output.clear();
	}
	return output;
}


