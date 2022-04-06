#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

/**
 * @brief Read the input.txt file and return a vector containing all the labyrinths
 * 
 * @param inputFile 
 * @return vector<vector<vector<string>>> 
 */
vector<vector<vector<string>>> getLabyrinth(fstream* inputFile){
    vector<vector<vector<string>>> ans;
    while(!inputFile->eof()){
        vector<vector<string>> temp;
        string s;
        while(getline(*inputFile, s)){
            cout << s << endl;
            vector<string> line;
            if (s == "/") {
                break;
            }
            for(int i=0; i<s.size(); i++){
                line.push_back(string(1, s[i]));
            }
            temp.push_back(line);
        }
        ans.push_back(temp);
    }
    inputFile->close();
    return ans;
}

/**
 * @brief Prints the vector given to the console
 * 
 * @param v 
 */
void printVector(vector<vector<string>> const &v){
    for(int i = 0; i < v.size(); i++){
        for(int j = 0; j < v[i].size(); j++){
            cout << v[i][j];
        }
        cout << endl;
    }
}

/**
 * @brief Writes the vector given to the output.txt file in append mode
 * 
 * @param v 
 * @param dist 
 */
void writeVector(vector<vector<string>> const &v, int dist){
    string filename = HOME + (string)"/output.txt";
    ofstream out(filename, ios_base::app);
    out << dist << '\n';
    for(int i = 0; i < v.size(); i++){
        for(int j = 0; j < v[i].size(); j++){
            out << v[i][j];
        }
        out << endl;
    }
    out.close();

}

/**
 * @brief Checks whether the labyrinth given has input and output points, if it doesn't then we can easily say that
 * it is invalid
 * 
 * @param v 
 * @return true 
 * @return false 
 */
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

/**
 * @brief Get a vector containing all the entry points (sources) to the labyrinth
 * 
 * @param v 
 * @return vector<pair<int,int>> 
 */
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

/**
 * @brief Get a vector containing all the exit points (destinations) to the labyrinth
 * 
 * @param v 
 * @return vector<pair<int,int>> 
 */
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

/**
 * @brief Checks whether the given square is safe to travel to i.e.
 * checks that it isn't a wall or out of bounds for the labyrinth
 * 
 * @param l 
 * @param v 
 * @param i 
 * @param j 
 * @return true 
 * @return false 
 */
bool isSafe(vector<vector<string>> &l, vector<vector<bool>> &v, int i, int j){
    if(i >= 0 && j >=0 && i<l.size() && j<l[0].size() && l[i][j] != "#" && v[i][j] != true){
        return true;
    }
    return false;
}

/**
 * @brief Backtracking function which uses recursion to find the longest path between a given source and a destination
 * for a labyrinth
 * 
 * @param labyrinth 
 * @param visited 
 * @param s_r 
 * @param s_c 
 * @param d_r 
 * @param d_c 
 * @param max_dist 
 * @param dist 
 */
void findLongestPath(vector<vector<string>> &labyrinth, vector<vector<bool>> &visited, int s_r, int s_c, int d_r, int d_c, int &max_dist, int dist){
    // Exit if it encounters a wall
    if(labyrinth[s_r][s_c] == "#"){
        return;
    }

    // Update the layrinth with the number only if this is the next step on the path
    if (dist > max_dist || max_dist == 0){
        labyrinth[s_r][s_c] = to_string(dist);
    }
    
    // if the destination is found, update 'max_dist'
    if(s_r == d_r && s_c == d_c){
        max_dist = max(dist, max_dist);
        return;
    }
    // Mark current location as visited
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
/**
 * @brief Function to iterate through all sources and destinations to determine longest path and write it to the output file
 * 
 * @param labyrinth 
 * @param destinations 
 * @param sources 
 */
void createOutputLabyrinth(vector<vector<string>> &labyrinth, vector<pair<int, int>> &destinations, vector<pair<int, int>> &sources){
    if(labyrinth.size() == 0 || destinations.size() == 0 || sources.size() == 0){
        return;
    }
    int m = labyrinth.size();
    int n = labyrinth[0].size();

    int write_dist = 0;
    vector<vector<string>> write_laby = labyrinth;

    for(int i=0; i < destinations.size(); i++){
        for(int j=0; j< sources.size(); j++){
            // Create visited vector of same size as labyrinth
            vector<vector<bool>> visited;
            visited.resize(m, vector<bool>(n));
            // Create temporary labyrinth which can be changed to show the path
            vector<vector<string>> t_laby = labyrinth;

            int max_dist = 0;

            findLongestPath(t_laby, visited, sources[i].first, sources[i].second,
            destinations[j].first, destinations[j].second, max_dist, 0);
            max_dist++;
            // Update the write variables if the new path is longer than the old one
            if(max_dist > write_dist){
                write_dist = max_dist;
                write_laby = t_laby;
            }
        }
    }
    if (write_dist == 1){
        writeVector(labyrinth, -1);
    }
    else{
        writeVector(write_laby, write_dist);
    }

}
/**
 * @brief Driver function
 * 
 * @return int 
 */
int main(){
    fstream inputFile;
    string filename = HOME + (string)"/input.txt";
    cout << filename << endl;
    inputFile.open(filename, ios::in);

    // Get Labyrinths in vector string representation from the input file
    vector<vector<vector<string>>> labyrinths = getLabyrinth(&inputFile);

    for(int i = 0; i < labyrinths.size(); i++){
        vector<vector<string>> labyrinth = labyrinths[i];
        // Print matrix
        printVector(labyrinth);

        // preliminary check for a valid labyrinth
        if(checkValid(labyrinth)){
            // Get a vector of possible destinations
            vector<pair<int, int>> destinations = getDestinations(labyrinth);

            // Get a vector of possible sources
            vector<pair<int, int>> sources = getSources(labyrinth);

            createOutputLabyrinth(labyrinth, destinations, sources);
        } else {
            // Labyrinth can't be navigated
            // Output invalid labyrinth
            writeVector(labyrinth, -1);
        }
    }

    inputFile.close();
    return 0;
}