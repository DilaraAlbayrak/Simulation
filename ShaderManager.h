#pragma once
#include <unordered_map>
#include <string>
#include <stdexcept> 
#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include <atlbase.h>
#include <memory>

class ShaderManager final
{
private:
    static std::unique_ptr<ShaderManager> _instance;
    ID3D11Device* _device;
    std::unordered_map<std::string, ID3D11VertexShader*> _vertexShaders;
    std::unordered_map<std::string, ID3D11PixelShader*> _pixelShaders;

    explicit ShaderManager(ID3D11Device* device);

public:
    ShaderManager(const ShaderManager&) = delete;  // Prevent copying
    ShaderManager& operator=(const ShaderManager&) = delete;
    ~ShaderManager() = default;
    static ShaderManager* getInstance(ID3D11Device* device);

    const HRESULT compileShaderFromFile(const WCHAR* szFileName, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut) const
    {
        DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
        dwShaderFlags |= D3DCOMPILE_DEBUG;

        // Disable optimizations to further improve shader debugging
        dwShaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

        CComPtr <ID3DBlob> pErrorBlob;
        const HRESULT hr = D3DCompileFromFile(szFileName, nullptr, nullptr, szEntryPoint, szShaderModel, dwShaderFlags, 0, ppBlobOut, &pErrorBlob);
        if (static_cast<HRESULT>(hr) < 0) {
            if (pErrorBlob)
                OutputDebugStringA(static_cast<const char*>(pErrorBlob->GetBufferPointer()));
        }
        return hr;
    }
};