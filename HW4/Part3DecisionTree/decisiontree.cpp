#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath> 
#include <unordered_map>
#include <limits>
#include <memory>
#include <algorithm> 

struct GameRecord {
    float character_x;
    float character_y;
    float enemy_x;
    float enemy_y;
    std::string enemy_action; // The label
    float distance; // Distance between character and enemy
};


std::vector<GameRecord> loadDataFromFile(const std::string& filename) {
    std::vector<GameRecord> records;
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        
        std::getline(file, line);

        // Read each data line and parse the values
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            GameRecord record;

            std::getline(ss, token, ',');
            record.character_x = std::stof(token);

            std::getline(ss, token, ',');
            record.character_y = std::stof(token);

            std::getline(ss, token, ',');
            record.enemy_x = std::stof(token);

            std::getline(ss, token, ',');
            record.enemy_y = std::stof(token);

            std::getline(ss, token, ',');
            record.enemy_action = token;

            std::getline(ss, token, ',');
            record.distance = std::stof(token);

            records.push_back(record);
        }

        file.close();
    } else {
        std::cerr << "Could not open file: " << filename << std::endl;
    }

    return records;
}

class DecisionTreeNode {
public:
    virtual ~DecisionTreeNode() {}
    virtual std::string classify(const GameRecord& record) = 0; 
};

// Leaf node representing a class (action)
class DecisionTreeLeaf : public DecisionTreeNode {
public:
    std::string label; 

    DecisionTreeLeaf(const std::string& label) : label(label) {}

    std::string classify(const GameRecord& record) override {
        return label;
    }
};

// Internal node representing a decision based on a feature
class DecisionTreeInternal : public DecisionTreeNode {
public:
    std::string feature; // The feature used to make a decision
    float threshold; // The threshold for the decision
    std::unique_ptr<DecisionTreeNode> leftChild; // Nodes to classify if value is below or equal to the threshold
    std::unique_ptr<DecisionTreeNode> rightChild; // Node if above threshold

    DecisionTreeInternal(const std::string& feature, float threshold,
                         std::unique_ptr<DecisionTreeNode> left,
                         std::unique_ptr<DecisionTreeNode> right)
        : feature(feature), threshold(threshold), leftChild(std::move(left)), rightChild(std::move(right)) {}

    std::string classify(const GameRecord& record) override {
        if (getFeatureValue(record) <= threshold) {
            return leftChild->classify(record);
        } else {
            return rightChild->classify(record);
        }
    }

    float getFeatureValue(const GameRecord& record) {
        if (feature == "character_x") {
            return record.character_x;
        } else if (feature == "character_y") {
            return record.character_y;
        } else if (feature == "enemy_x") {
            return record.enemy_x;
        } else if (feature == "enemy_y") {
            return record.enemy_y;
        } else if (feature == "distance") {
            return record.distance;
        }
        throw std::runtime_error("Unknown feature");
    }
};

class DecisionTree {
public:
    std::unique_ptr<DecisionTreeNode> root; // Root node of the decision tree

    DecisionTree(std::unique_ptr<DecisionTreeNode> root) : root(std::move(root)) {}

    std::string classify(const GameRecord& record) {
        return root->classify(record);
    }
};

// Calculate the entropy of a distribution
float entropy(const std::unordered_map<std::string, int>& counts, int total) {
    float entropy = 0.0f;
    for (const auto& kvp : counts) {
        if (kvp.second > 0) {
            float probability = static_cast<float>(kvp.second) / total;
            entropy -= probability * std::log2(probability);
        }
    }
    return entropy;
}

// Calculate the information gain for a given feature and threshold
float calculateInformationGain(const std::vector<GameRecord>& records, const std::string& feature, float threshold) {
    std::unordered_map<std::string, int> leftCounts;
    std::unordered_map<std::string, int> rightCounts;
    int leftTotal = 0;
    int rightTotal = 0;

    for (const auto& record : records) {
        float featureValue;
        if (feature == "character_x") {
            featureValue = record.character_x;
        } else if (feature == "character_y") {
            featureValue = record.character_y;
        } else if (feature == "enemy_x") {
            featureValue = record.enemy_x;
        } else if (feature == "enemy_y") {
            featureValue = record.enemy_y;
        } else if (feature == "distance") {
            featureValue = record.distance;
        }

        if (featureValue <= threshold) {
            leftCounts[record.enemy_action]++;
            leftTotal++;
        } else {
            rightCounts[record.enemy_action]++;
            rightTotal++;
        }
    }

    float totalEntropy = entropy(leftCounts, leftTotal) * (static_cast<float>(leftTotal) / (leftTotal + rightTotal)) +
                         entropy(rightCounts, rightTotal) * (static_cast<float>(rightTotal) / (leftTotal + rightTotal));

    float originalEntropy = entropy(leftCounts, leftTotal + rightTotal);
    return originalEntropy - totalEntropy;
}

// Find the best feature and threshold to split on
std::pair<std::string, float> findBestSplit(const std::vector<GameRecord>& records) {
    std::vector<std::string> features = {"character_x", "character_y", "enemy_x", "enemy_y", "distance"};
    float bestGain = -std::numeric_limits<float>::infinity();
    std::string bestFeature;
    float bestThreshold;

    for (const auto& feature : features) {
        // Try different thresholds
        std::vector<float> values;
        for (const auto& record : records) {
            float value = (feature == "character_x" ? record.character_x
                          : feature == "character_y" ? record.character_y
                          : feature == "enemy_x" ? record.enemy_x
                          : feature == "enemy_y" ? record.enemy_y
                          : record.distance);

            values.push_back(value);
        }

        // Sort and get unique thresholds to try
        std::sort(values.begin(), values.end());
        std::vector<float> uniqueThresholds;
        uniqueThresholds.push_back(values.front());

        for (size_t i = 1; i < values.size(); ++i) {
            if (values[i] != uniqueThresholds.back()) {
                uniqueThresholds.push_back(values[i]);
            }
        }

        for (float threshold : uniqueThresholds) {
            float gain = calculateInformationGain(records, feature, threshold);
            if (gain > bestGain) {
                bestGain = gain;
                bestFeature = feature;
                bestThreshold = threshold;
            }
        }
    }

    return {bestFeature, bestThreshold};
}

// Recursive function to build the decision tree
std::unique_ptr<DecisionTreeNode> buildTree(const std::vector<GameRecord>& records) {
    std::cout<<"Building Tree"<<std::endl;
    if (records.empty()) {
        throw std::runtime_error("Cannot build tree with no records");
    }

    // Check if all actions are the same
    std::unordered_map<std::string, int> actionCounts;
    for (const auto& record : records) {
        actionCounts[record.enemy_action]++;
    }

    if (actionCounts.size() == 1) { // All records have the same action
        return std::make_unique<DecisionTreeLeaf>(records.front().enemy_action);
    }

    // Find the best feature and threshold to split on
    auto bestSplit = findBestSplit(records);

    // Divide records based on the best feature and threshold
    std::vector<GameRecord> leftRecords;
    std::vector<GameRecord> rightRecords;

    for (const auto& record : records) {
        float value = (bestSplit.first == "character_x" ? record.character_x
                      : bestSplit.first == "character_y" ? record.character_y
                      : bestSplit.first == "enemy_x" ? record.enemy_x
                      : bestSplit.first == "enemy_y" ? record.enemy_y
                      : record.distance);

        if (value <= bestSplit.second) {
            leftRecords.push_back(record);
        } else {
            rightRecords.push_back(record);
        }
    }

    // Recursively build the tree
    std::unique_ptr<DecisionTreeNode> leftChild = buildTree(leftRecords);
    std::unique_ptr<DecisionTreeNode> rightChild = buildTree(rightRecords);

    return std::make_unique<DecisionTreeInternal>(bestSplit.first, bestSplit.second, std::move(leftChild), std::move(rightChild));
}

// Main function
int main() {
    // Load the game records from the CSV file (training data)
    auto records = loadDataFromFile("game_records.csv");

    // Build the decision tree
    std::unique_ptr<DecisionTreeNode> tree = buildTree(records);

    // Create the decision tree object
DecisionTree decisionTree(std::move(tree));

    // Load test data
    auto testRecords = loadDataFromFile("game_records_test.csv");

    // Classify test data and calculate accuracy
    int correctPredictions = 0;
    int totalPredictions = 0;

    // Display the predicted and actual actions
    for (const auto& record : testRecords) {
        std::string predictedAction = decisionTree.classify(record);
        std::string actualAction = record.enemy_action;

        // Display the predicted and actual actions
        std::cout << "Predicted: " << predictedAction << ", Actual: " << actualAction << std::endl;

        if (predictedAction == actualAction) {  // Check if prediction matches the actual action
            correctPredictions++;
        }
        totalPredictions++;
    }

    // Calculate and display accuracy
    float accuracy = static_cast<float>(correctPredictions) / totalPredictions;
    std::cout << "Accuracy: " << (accuracy * 100) << "%" << std::endl;

    return 0;
}
