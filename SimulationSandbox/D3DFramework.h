#pragma once
// Simulation template, based on the Microsoft DX11 tutorial 04

#include <windows.h>
#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include <atlbase.h>
#include <fstream>

using namespace DirectX;

#define COMPILE_CSO

//--------------------------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------------------------
struct SimpleVertex
{
	XMFLOAT3 Pos;
	XMFLOAT4 Color;
};


struct ConstantBuffer
{
	XMMATRIX mWorld;
	XMMATRIX mView;
	XMMATRIX mProjection;
};


class D3DFramework final {

	HINSTANCE _hInst = nullptr;
	HWND _hWnd = nullptr;
	D3D_DRIVER_TYPE _driverType = D3D_DRIVER_TYPE_NULL;
	D3D_FEATURE_LEVEL _featureLevel = D3D_FEATURE_LEVEL_11_1;
	CComPtr <ID3D11Device> _pd3dDevice;
	CComPtr <ID3D11Device1> _pd3dDevice1;
	CComPtr <ID3D11DeviceContext> _pImmediateContext;
	CComPtr <ID3D11DeviceContext1> _pImmediateContext1;
	CComPtr <IDXGISwapChain1> _swapChain;
	CComPtr <IDXGISwapChain1> _swapChain1;
	CComPtr <ID3D11RenderTargetView> _pRenderTargetView;
	CComPtr <ID3D11VertexShader> _pVertexShader;
	CComPtr <ID3D11PixelShader> _pPixelShader;
	CComPtr <ID3D11InputLayout> _pVertexLayout;
	CComPtr <ID3D11Buffer> _pVertexBuffer;
	CComPtr <ID3D11Buffer> _pIndexBuffer;
	CComPtr <ID3D11Buffer> _pConstantBuffer;
	XMMATRIX _World = {};
	XMMATRIX _View = {};
	XMMATRIX _Projection = {};

	float _rotation = 0.0f;
	
	static std::unique_ptr<D3DFramework> _instance;
public:

	D3DFramework() = default;
	D3DFramework(D3DFramework&) = delete;
	D3DFramework(D3DFramework&&) = delete;
	D3DFramework operator=(const D3DFramework&) = delete;
	D3DFramework operator=(const D3DFramework&&) = delete;
	~D3DFramework();
	
	static D3DFramework& getInstance() { return *_instance; }

	// callback function that Windows calls whenever an event occurs for the window (e.g., mouse clicks, key presses)
	// Windows expects this function to have a specific signature and does not pass an instance of the class to it
	static LRESULT CALLBACK wndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	HRESULT initWindow(HINSTANCE hInstance, int nCmdShow);
	// utility function that doesn't require an instance of the class
	static HRESULT compileShaderFromFile(const WCHAR* szFileName, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut);
	HRESULT initDevice();
	void render();
};