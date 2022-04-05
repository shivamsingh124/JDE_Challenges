#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

vector<vector<string>> getLabyrinth(fstream* inputFile){
    vector<vector<string>> ans;
    while(inputFile->is_open()){
        string s;
        while(getline(*inputFile, s)){
            vector<string> line;
            for(int i=0; i<s.size(); i++){
                line.push_back(string(1, s[i]));
            }
            ans.push_back(line);
        }
        inputFile->close();
    }
    return ans;
}

void printVector(vector<vector<string>> const &v){
    for(int i = 0; i < v.size(); i++){
        for(int j = 0; j < v[i].size(); j++){
            cout << v[i][j];
        }
        cout << endl;
    }

}

bool checkValid(vector<vector<string>> &v){
    bool top = false;
    bool bottom = false;
    for(int i = 0; i < v[0].size(); i++){
        if(v[0][i] == "."){
            top = true;
            break;
        }
    }

    for(int j = 0; j < v[v.size()-1].size(); j++){
        if(v[v.size()-1][j] == "."){
            bottom = true;
            break;
        }
    }

    return (top && bottom);

}

vector<pair<int,int>> getSources(vector<vector<string>> const &v){
    vector<pair<int,int>> sourceList;
    for(int i = 0; i < v[0].size(); i++){
        if(v[0][i] == "."){
            // cout << 0 << " " << i << endl;
            sourceList.push_back(make_pair(0, i));
        }
    }
    return sourceList;
}

vector<pair<int,int>> getDestinations(vector<vector<string>> const &v){
    vector<pair<int,int>> destinationList;
    for(int i = 0; i < v[v.size()-1].size(); i++){
        if(v[v.size()-1][i] == "."){
            // cout << v.size()-1 << " " << i << endl;
            destinationList.push_back(make_pair(v.size()-1, i));
        }
    }
    return destinationList;
}

bool isSafe(vector<vector<string>> &l, vector<vector<bool>> &v, int i, int j){
    if(i >= 0 && j >=0 && i<l.size() && j<l[0].size() && l[i][j] != "#" && v[i][j] != true){
        return true;
    }
    return false;
}

void findLongestPath(vector<vector<string>> &labyrinth, vector<vector<bool>> &visited, int s_r, int s_c, int d_r, int d_c, int &max_dist, int dist){
    if(labyrinth[s_r][s_c] == "#"){
        return;
    }

    if (dist > max_dist || max_dist == 0){
        labyrinth[s_r][s_c] = to_string(dist);
    }
    // if the destination is found, update 'max_dist'
    if(s_r == d_r && s_c == d_c){
        max_dist = max(dist, max_dist);
        return;
    }

    visited[s_r][s_c] = true;

    //iterate through the remaining 8 directions
    // top
    if(isSafe(labyrinth, visited, s_r - 1, s_c)){
        findLongestPath(labyrinth, visited, s_r - 1, s_c, d_r, d_c, 
        max_dist, dist + 1);
    }

    // top-right
    if(isSafe(labyrinth, visited, s_r - 1, s_c + 1)){
        findLongestPath(labyrinth, visited, s_r - 1, s_c + 1, d_r, d_c, 
        max_dist, dist + 1);
    }

    // top-left
    if(isSafe(labyrinth, visited, s_r - 1, s_c - 1)){
        findLongestPath(labyrinth, visited, s_r - 1, s_c - 1, d_r, d_c, 
        max_dist, dist + 1);
    }

    // right
    if(isSafe(labyrinth, visited, s_r, s_c + 1)){
        findLongestPath(labyrinth, visited, s_r, s_c + 1, d_r, d_c, 
        max_dist, dist + 1);
    } 

    // left
    if(isSafe(labyrinth, visited, s_r, s_c - 1)){
        findLongestPath(labyrinth, visited, s_r, s_c - 1, d_r, d_c, 
        max_dist, dist + 1);
    }

    // bottom
    if(isSafe(labyrinth, visited, s_r + 1, s_c)){
        findLongestPath(labyrinth, visited, s_r + 1, s_c, d_r, d_c, 
        max_dist, dist + 1);
    }

    // bottom-left
    if(isSafe(labyrinth, visited, s_r + 1, s_c - 1)){
        findLongestPath(labyrinth, visited, s_r + 1, s_c - 1, d_r, d_c, 
        max_dist, dist + 1);
    }

    // bottom-right
    if(isSafe(labyrinth, visited, s_r + 1, s_c + 1)){
        findLongestPath(labyrinth, visited, s_r + 1, s_c + 1, d_r, d_c, 
        max_dist, dist + 1);
    }

    visited[s_r][s_c] = false;
}

void createResultantLabyrinth(vector<vector<string>> &labyrinth, vector<pair<int, int>> &destinations, vector<pair<int, int>> &sources){
    if(labyrinth.size() == 0 || destinations.size() == 0 || sources.size() == 0){
        return;
    }
    int m = labyrinth.size();
    int n = labyrinth[0].size();

    for(int i=0; i < destinations.size(); i++){
        for(int j=0; j< sources.size(); j++){
            vector<vector<bool>> visited;
            vector<vector<string>> t_laby = labyrinth;
            visited.resize(m, vector<bool>(n));

            int max_dist = 0;

            findLongestPath(t_laby, visited, sources[i].first, sources[i].second,
            destinations[j].first, destinations[j].second, max_dist, 0);
            cout << "Maximum distance for " << i << " " << j <<" is " << max_dist + 1 << endl;
            // printVector(t_laby);
        }
        
    }

}

int main(){
    fstream inputFile;
    inputFile.open("input.txt", ios::in);
    string s;
    getline(inputFile, s);
    // Remove newline character from the string
    if (!s.empty() && s[s.length()-1] == '\n') {
        s.erase(s.length()-1);
    }
    int t = stoi(s);
    while(t-- > 0){
        // Create Labyrinth in vector string representation
        vector<vector<string>> labyrinth = getLabyrinth(&inputFile);
        // Print matrix
        printVector(labyrinth);

        // Get a vector of possible destinations
        vector<pair<int, int>> destinations = getDestinations(labyrinth);
        // Get a vector of possible sources
        vector<pair<int, int>> sources = getSources(labyrinth);
        // preliminary check for a valid labyrinth
        if(checkValid(labyrinth)){
            createResultantLabyrinth(labyrinth, destinations, sources);
        }
    }
    return 0;
}