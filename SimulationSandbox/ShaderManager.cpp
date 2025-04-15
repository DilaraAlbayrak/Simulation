#include "ShaderManager.h"

// Initialize the static instance to nullptr
std::unique_ptr<ShaderManager> ShaderManager::_instance = nullptr;

ShaderManager::ShaderManager(ID3D11Device* device) : _device(device) {}

ShaderManager* ShaderManager::getInstance(ID3D11Device* device = nullptr) {
    if (!_instance) {
        if (!device) {
            throw std::runtime_error("ShaderManager must be initialized with a valid device.");
        }
        //_instance = std::make_unique<ShaderManager>(device);
        // make_unique did not work with private constructor
        _instance = std::unique_ptr<ShaderManager>(new ShaderManager(device));
    }
    return _instance.get();
}