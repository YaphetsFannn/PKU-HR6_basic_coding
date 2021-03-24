#include <vector>
#include <string>
using namespace std;

#define MAXLAYERS 150

class NN {
public:
	NN (){}
	NN (const string net_path);
	~NN (){
		//delete n_layerNodes;
		//weight.clear();
	}
	vector<float> predict(vector<float> input);
	float sigmoid(float z);
public:
	int n_layers;
	int n_layerNodes[MAXLAYERS];
	vector<vector<float> > weight;
};
