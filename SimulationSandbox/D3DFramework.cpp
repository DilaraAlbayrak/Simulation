#include "D3DFramework.h"
#include <directxcolors.h>
#include <vector>
#include "Resource.h"
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include "Scenario1.h"
#include "Scenario2.h"
#include "Scenario3.h"
#include "Scenario4.h"
#include "Scenario5.h"
#include "TestScenario1.h"
#include "TestScenario2.h"
#include "TestScenario3.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

std::unique_ptr<D3DFramework> D3DFramework::_instance = std::make_unique<D3DFramework>();
float D3DFramework::time = 0.0f;

//--------------------------------------------------------------------------------------
// Called every time the application receives a message
//--------------------------------------------------------------------------------------
LRESULT CALLBACK D3DFramework::wndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, message, wParam, lParam))
		return true;

	switch (message) {
	case WM_LBUTTONDOWN:
		_instance->_isMouseDown = true;
		GetCursorPos(&_instance->_lastMousePos);
		break;
	case WM_LBUTTONUP:
		_instance->_isMouseDown = false;
		break;
	case WM_MOUSEMOVE:
		if (_instance->_isMouseDown)
		{
			POINT currentMousePos;
			GetCursorPos(&currentMousePos);
			
			float dx = (currentMousePos.x - _instance->_lastMousePos.x) * _instance->_sensitivity;
			float dy = (currentMousePos.y - _instance->_lastMousePos.y) * _instance->_sensitivity;

			_instance->_yaw += dx;
			_instance->_pitch -= dy;

			float epsilon = 0.01f;
			if (_instance->_pitch > XM_PIDIV2 - epsilon) _instance->_pitch = XM_PIDIV2 - epsilon;
			if (_instance->_pitch < -XM_PIDIV2 + epsilon) _instance->_pitch = -XM_PIDIV2 + epsilon;
			
			_instance->_camera.rotate(_instance->_yaw, _instance->_pitch);

			_instance->_lastMousePos = currentMousePos;
		}
		break;

	case WM_MOUSEWHEEL:
	{
		short wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		float zoomAmount = 1.0f;
		if (wheelDelta > 0)  // Scroll up, Zoom in
			zoomAmount += _instance->_zoomSensitivity;
		else if (wheelDelta < 0) // Scroll down, Zoom out
			zoomAmount -= _instance->_zoomSensitivity;

		_instance->_camera.zoom *= zoomAmount;
		_instance->_camera.updateViewProjection();
		break;
	}

	case WM_PAINT:
		PAINTSTRUCT ps;
		BeginPaint(hWnd, &ps);
		EndPaint(hWnd, &ps);
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

//--------------------------------------------------------------------------------------
// Register class and create window
//--------------------------------------------------------------------------------------
HRESULT D3DFramework::initWindow(HINSTANCE hInstance, int nCmdShow) {
	// Register class
	WNDCLASSEX wcex;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = wndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, reinterpret_cast<LPCTSTR>(IDI_SIMULATION));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
	wcex.lpszMenuName = nullptr;
	wcex.lpszClassName = L"Starter Template";
	wcex.hIconSm = LoadIcon(wcex.hInstance, reinterpret_cast<LPCTSTR>(IDI_SIMULATION));
	if (!RegisterClassEx(&wcex))
		return E_FAIL;

	// Create window
	_hInst = hInstance;
	RECT rc = { 0, 0, _windowWidth, _windowHeight };
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
	_hWnd = CreateWindow(L"Starter Template", L"Networked AntiGravity Chamber",
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hInstance,
		nullptr);
	if (!_hWnd)
		return E_FAIL;

	ShowWindow(_hWnd, nCmdShow);

	return S_OK;
}

//--------------------------------------------------------------------------------------
// Create Direct3D device and swap chain
//--------------------------------------------------------------------------------------
HRESULT D3DFramework::initDevice()
{
	HRESULT hr = static_cast<HRESULT>(S_OK);

	UINT createDeviceFlags = 0;
#ifdef _DEBUG
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	// Supported driver types
	D3D_DRIVER_TYPE driverTypes[] = {
		D3D_DRIVER_TYPE_HARDWARE,
		D3D_DRIVER_TYPE_WARP,
		D3D_DRIVER_TYPE_REFERENCE,
	};
	auto numDriverTypes = ARRAYSIZE(driverTypes);

	// Supported feature levels
	D3D_FEATURE_LEVEL featureLevels[] = {
		D3D_FEATURE_LEVEL_11_1,
		D3D_FEATURE_LEVEL_11_0,
		D3D_FEATURE_LEVEL_10_1,
		D3D_FEATURE_LEVEL_10_0,
	};
	auto numFeatureLevels = static_cast<UINT>(ARRAYSIZE(featureLevels));

	// Attempt to create the device and context
	for (UINT driverTypeIndex = 0; driverTypeIndex < numDriverTypes; ++driverTypeIndex) {
		_driverType = driverTypes[driverTypeIndex];
		hr = D3D11CreateDevice(
			nullptr, _driverType, nullptr, D3D11_CREATE_DEVICE_DEBUG,
			featureLevels, numFeatureLevels, D3D11_SDK_VERSION,
			&_pd3dDevice, &_featureLevel, &_pImmediateContext);

		if (hr == static_cast<HRESULT>(E_INVALIDARG)) {
			// Retry without D3D_FEATURE_LEVEL_11_1 if unsupported
			hr = D3D11CreateDevice(
				nullptr, _driverType, nullptr, D3D11_CREATE_DEVICE_DEBUG,
				&featureLevels[1], numFeatureLevels - 1, D3D11_SDK_VERSION,
				&_pd3dDevice, &_featureLevel, &_pImmediateContext);
		}

		if (SUCCEEDED(hr))
			break;
	}

	if (FAILED(hr))
		return hr;

	// Obtain DXGI factory
	CComPtr<IDXGIFactory1> dxgiFactory;
	{
		CComPtr<IDXGIDevice> dxgiDevice;
		hr = _pd3dDevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice));
		if (SUCCEEDED(hr)) {
			CComPtr<IDXGIAdapter> adapter;
			hr = dxgiDevice->GetAdapter(&adapter);
			if (SUCCEEDED(hr)) {
				hr = adapter->GetParent(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory));
			}
		}
	}
	if (FAILED(hr))
		return hr;

	// Create swap chain
	CComPtr<IDXGIFactory2> dxgiFactory2;
	hr = dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));

	// DirectX 11.1 or later
	hr = _pd3dDevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&_pd3dDevice1));
	if (SUCCEEDED(hr)) {
		static_cast<void>(_pImmediateContext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&_pImmediateContext1)));
	}

	DXGI_SWAP_CHAIN_DESC1 sd{};
	sd.Width = _windowWidth;
	sd.Height = _windowHeight;
	sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.SampleDesc.Count = 1;
	sd.SampleDesc.Quality = 0;
	sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.BufferCount = 2;

	hr = dxgiFactory2->CreateSwapChainForHwnd(_pd3dDevice, _hWnd, &sd, nullptr, nullptr, &_swapChain1);
	if (SUCCEEDED(hr)) {
		hr = _swapChain1->QueryInterface(__uuidof(IDXGISwapChain), reinterpret_cast<void**>(&_swapChain));
	}

	// Disable Alt+Enter fullscreen shortcut
	dxgiFactory->MakeWindowAssociation(_hWnd, DXGI_MWA_NO_ALT_ENTER);

	if (FAILED(hr))
		return hr;

	// Create render target view
	CComPtr<ID3D11Texture2D> pBackBuffer;
	hr = _swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer));
	if (FAILED(hr))
		return hr;

	hr = _pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &_pRenderTargetView);
	if (FAILED(hr))
		return hr;

	_pImmediateContext->OMSetRenderTargets(1, &_pRenderTargetView.p, nullptr);

	// Setup viewport
	D3D11_VIEWPORT vp;
	vp.Width = static_cast<FLOAT>(_windowWidth);
	vp.Height = static_cast<FLOAT>(_windowHeight);
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	_pImmediateContext->RSSetViewports(1, &vp);

	// Create constant buffer for camera matrices
	D3D11_BUFFER_DESC cbDesc = {};
	cbDesc.Usage = D3D11_USAGE_DEFAULT;
	cbDesc.ByteWidth = sizeof(ConstantBufferCamera);
	cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;

	hr = _pd3dDevice->CreateBuffer(&cbDesc, nullptr, &_cameraConstantBuffer);
	if (FAILED(hr)) {
		return hr;
	}

	// Create depth stencil buffer
	D3D11_TEXTURE2D_DESC depthStencilDesc = {};
	depthStencilDesc.Width = _windowWidth;
	depthStencilDesc.Height = _windowHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.SampleDesc.Count = 1;
	depthStencilDesc.SampleDesc.Quality = 0;
	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

	hr = _pd3dDevice->CreateTexture2D(&depthStencilDesc, nullptr, &_depthStencilBuffer);
	if (FAILED(hr)) return hr;

	hr = _pd3dDevice->CreateDepthStencilView(_depthStencilBuffer, nullptr, &_depthStencilView);
	if (FAILED(hr)) return hr;

	//// Create and store depth stencil state
	//D3D11_DEPTH_STENCIL_DESC dsDesc = {};
	//dsDesc.DepthEnable = TRUE;
	//dsDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
	//dsDesc.DepthFunc = D3D11_COMPARISON_LESS;

	//hr = _pd3dDevice->CreateDepthStencilState(&dsDesc, &_depthStencilState);
	//if (FAILED(hr)) return hr;

	// create rasterizer state to disable backface culling
	/*D3D11_RASTERIZER_DESC rasterizerDesc = {};
	rasterizerDesc.CullMode = D3D11_CULL_BACK;
	rasterizerDesc.FillMode = D3D11_FILL_SOLID;
	rasterizerDesc.DepthBias = 1;
	rasterizerDesc.SlopeScaledDepthBias = 1.0f;
	rasterizerDesc.DepthBiasClamp = 0.0f;
	hr = _pd3dDevice->CreateRasterizerState(&rasterizerDesc, &_rasterizerState);
	if (FAILED(hr)) {
		return hr;
	}
	_pImmediateContext->RSSetState(_rasterizerState);*/

	// Initialize the camera
	//initCamera();
	initImGui();

	return S_OK;
}

void D3DFramework::initImGui()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplWin32_Init(_hWnd);
	ImGui_ImplDX11_Init(_pd3dDevice, _pImmediateContext);
}

void D3DFramework::renderImGui() {
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("Scenario"))
		{
			if (ImGui::MenuItem("Scenario 1", nullptr, false)) {
				setScenario(std::make_unique<Scenario1>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Scenario 2", nullptr, false)) {
				setScenario(std::make_unique<Scenario2>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Scenario 3", nullptr, false)) {
				setScenario(std::make_unique<Scenario3>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Scenario 4", nullptr, false)) {
				setScenario(std::make_unique<Scenario4>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Scenario 5", nullptr, false)) {
				setScenario(std::make_unique<Scenario5>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Test Scenario 1", nullptr, false)) {
				setScenario(std::make_unique<TestScenario1>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Test Scenario 2", nullptr, false)) {
				setScenario(std::make_unique<TestScenario2>(_pd3dDevice, _pImmediateContext));
			}
			if (ImGui::MenuItem("Test Scenario 3", nullptr, false)) {
				setScenario(std::make_unique<TestScenario3>(_pd3dDevice, _pImmediateContext));
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}

	if (_scenario)
	//if (false)
	{
		ImGui::Begin("Statistics");
		ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
		ImGui::Text("dt: %.4f", deltaTime);
		ImGui::End();

		if (ImGui::BeginMainMenuBar()) {
			if (ImGui::BeginMenu("Camera")) {
				if (ImGui::MenuItem("Reset", nullptr, false, true)) {
					_instance->_camera.initCamera();
				}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		static int selectedTimeCoefIndex = 2;
		static int selectedTimestepIndex = 1;

		ImGui::Begin("Scene Controls");
		ImGui::PushItemWidth(100);
		if (ImGui::SliderInt("Time coefficient:", &selectedTimeCoefIndex, 0, 3, ""))
		{
			// Ensure the index stays within the range of the array
			selectedTimeCoefIndex = (selectedTimeCoefIndex < 0) ? 0 : (selectedTimeCoefIndex > 3) ? 3 : selectedTimeCoefIndex;
			deltaTimeFactor = deltaTimeFactors[selectedTimeCoefIndex];
		}
		if (ImGui::SliderInt("Fixed timestep:", &selectedTimestepIndex, 0, 3, ""))
		{
			// Ensure the index stays within the range of the array
			selectedTimestepIndex = (selectedTimestepIndex < 0) ? 0 : (selectedTimestepIndex > 3) ? 3 : selectedTimestepIndex;
			fixedTimestep = fixedTimesteps[selectedTimestepIndex] * deltaTimeFactor;
			numMaxStep = maxSteps[selectedTimestepIndex];
		}
		ImGui::SameLine();
		ImGui::Text("%.3f", fixedTimestep);
		ImGui::PopItemWidth();
		ImGui::End();

		//_scenario->ImGuiMainMenu();
	}
	else
	{
		ImGui::Begin("Colour Picker");

		float colour[3] = { _bgColour.x, _bgColour.y, _bgColour.z };

		ImGui::Text("Set background colour:");
		if (ImGui::ColorEdit3("Colour", colour))
		{
			DirectX::XMFLOAT4 newColour = { colour[0], colour[1], colour[2], 1.0f };
			{
				_bgColour = newColour;
			}
		}
		ImGui::End();
	}
	
	ImGui::Render();
	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}


//--------------------------------------------------------------------------------------
// Clean up the objects we've created
//--------------------------------------------------------------------------------------
D3DFramework::~D3DFramework() {
	try {
		if (_pImmediateContext)
			_pImmediateContext->ClearState();
	}
	catch (...) {

	}
}


//--------------------------------------------------------------------------------------
// Render a frame
//--------------------------------------------------------------------------------------
void D3DFramework::render()
{
	static float currentTime = 0.0f;
	// Update our time	
	if (_driverType == D3D_DRIVER_TYPE_REFERENCE)
	{
		time += static_cast<float>(XM_PI) * 0.0125f;
	}
	else
	{
		static ULONGLONG timeStart = 0;
		const ULONGLONG timeCur = GetTickCount64();
		if (timeStart == 0)
			timeStart = timeCur;
		time = (timeCur - timeStart) / 1000.0f;
	}
	deltaTime = (time - currentTime) * deltaTimeFactor;
	currentTime = time;

	_pImmediateContext->OMSetDepthStencilState(_depthStencilState, 1);
	//_pImmediateContext->OMSetRenderTargets(1, &_pRenderTargetView.p, nullptr);
	_pImmediateContext->OMSetRenderTargets(1, &_pRenderTargetView.p, _depthStencilView.p);
	//
	// Clear the back buffer
	//
	float clearColour[4] = { _bgColour.x, _bgColour.y, _bgColour.z, _bgColour.w };
	_pImmediateContext->ClearRenderTargetView(_pRenderTargetView, clearColour);
	_pImmediateContext->ClearDepthStencilView(_depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	// Pass camera matrices to the constant buffer
	const ConstantBufferCamera cbc{
		DirectX::XMMatrixTranspose(_camera.view),
		DirectX::XMMatrixTranspose(_camera.projection),
		{
			DirectX::XMVectorGetX(_camera.eye),
			DirectX::XMVectorGetY(_camera.eye),
			DirectX::XMVectorGetZ(_camera.eye),
			time
		}
	};

	if (_scenario)
	{
		_scenario->onFrameUpdate();

		cumulativeTime += deltaTime;
		int physicsSteps = 0;

		while (cumulativeTime >= fixedTimestep && physicsSteps <= numMaxStep)
		{
			_scenario->onUpdate(fixedTimestep);
			cumulativeTime -= fixedTimestep;
			++physicsSteps;
		}

		_scenario->renderObjects();
	}
	
	_pImmediateContext->UpdateSubresource(_cameraConstantBuffer, 0, nullptr, &cbc, 0, 0);
	_pImmediateContext->VSSetConstantBuffers(0, 1, &_cameraConstantBuffer.p); // register b0

	renderImGui();
	//_swapChain->Present(1, 0);
	_swapChain->Present(0, 0);
}