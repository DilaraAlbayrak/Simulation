#include "SJGLoader.h"
#include <fstream>
#include <sstream>
#include <vector>

bool SJGLoader::Load(const std::string& filename, std::vector<Vertex>& vertices, std::vector<int>& indices)
{
    std::ifstream file(filename);
    if (!file)
    {
        return false;
    }

    std::string line;
    bool readingVertices = false, readingIndices = false;

    while (std::getline(file, line))
    {
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        std::istringstream ss(line);
        std::string firstWord;
        ss >> firstWord;

        if (firstWord == "Vertex")
        {
            readingVertices = true;
            readingIndices = false;
            std::getline(file, line); 
            continue;
        }

        if (firstWord == "Index")
        {
            readingVertices = false;
            readingIndices = true;
            std::getline(file, line);
            continue;
        }

        if (readingVertices)
        {
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream linestream(line);

            while (linestream >> token)
            {
                tokens.push_back(token);
            }

            if (tokens.size() == 6) // x, y, z, nx, ny, nz
            {
                Vertex vertex;
                vertex.position.x = std::stof(tokens[0]);
                vertex.position.y = std::stof(tokens[1]);
                vertex.position.z = std::stof(tokens[2]);
                vertex.normal.x = std::stof(tokens[3]);
                vertex.normal.y = std::stof(tokens[4]);
                vertex.normal.z = std::stof(tokens[5]);
                vertices.push_back(vertex);
            }
        }
        else if (readingIndices)
        {
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream linestream(line);

            while (linestream >> token)
            {
                tokens.push_back(token);
            }

            if (tokens.size() == 3) // v1, v2, v3
            {
                indices.push_back(std::stoi(tokens[0]));
                indices.push_back(std::stoi(tokens[1]));
                indices.push_back(std::stoi(tokens[2]));
            }
        }
    }

    file.close();
    return true;
}