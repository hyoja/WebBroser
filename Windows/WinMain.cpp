#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#include <windows.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <codecvt>
#include <tchar.h>
#include <iostream>
#include <CommCtrl.h>
#include <regex>
#include <fstream>
#include <GdiPlus.h>
#include <locale>
#include <commctrl.h>
#include <atlconv.h>

using namespace std;
using namespace Gdiplus;

#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")
#pragma comment(lib, "ws2_32.lib") 
#pragma comment(lib, "gdiplus")
#pragma comment(lib, "comctl32.lib")

#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#pragma endregion

#define MAXLENGTH 1024
#define STATUSCODELENGTH 7

string statusCode[] =
{
"301 Moved Permanently",
"302 Moved Temporarily",
"302 Found",
"400 Bad Request",
"403 Forbidden",
"404 Not Found",
"501 Not Implemented", };

WNDPROC OldEditProc;
WNDPROC hyperLinkProc;
HWND hLink;
HWND textfieldWindowHandle, mainWindowHandle;
HWND backButtonWindowHandle, frontButtonWindowHandle, refleshButtonWindowHandle;
HINSTANCE hInst;

LRESULT CALLBACK MainWindowProcedure(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK URLTextFieldProcedure(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK ClickHyperLinkProcedure(HWND handleWindow, UINT message, WPARAM wParam, LPARAM lParam);

void CommunicateServerToClient(char* textField);
string replaceAll(const string &str, const string &pattern, const string &replace);

class Parser
{
private:
public:
	string SplitTag(char* token, char delimiter, bool type, bool isKeepGoing)
	{
		string str;
		bool isStart = false;
		char secondDelimiter = ' ';

		if (delimiter == '<')
		{
			secondDelimiter = '>';
		}
		else if (delimiter == '\"')
		{
			secondDelimiter = '\"';
		}

		for (unsigned int i = 0; i < strlen(token); i++)
		{
			if (token[i] == delimiter && isStart == false)
			{
				isStart = true;
			}
			else if (token[i] == secondDelimiter && isStart == true)
			{
				isStart = false;
				if (isKeepGoing == false)
				{
					break;
				}
			}
			else if (isStart == type)
			{
				str += token[i];
			}
		}
		return str;
	}
	vector<string> GetParsedHTML(vector<string> receivedHTML)
	{
		vector<string> parsedHTML;

		if (receivedHTML.size() < 1)
		{
			receivedHTML.push_back("ERR_NAME_NOT_RESOLVED !");
			receivedHTML.push_back("사랑합니다 고객님 !");
			return receivedHTML;
		}
		else
		{
			for (int i = 0; i < STATUSCODELENGTH; i++)
			{
				if (strstr(receivedHTML[0].c_str(), statusCode[i].c_str()) > 0)
				{
					return receivedHTML;
				}
			}
		}

		for (unsigned int i = 0; i < receivedHTML.size(); i++)
		{
			string str(receivedHTML[i]);
			str = replaceAll(str, "\r", "");
			str = replaceAll(str, "\t", "");
			str = replaceAll(str, "\n", "");

			while (strstr(str.c_str(), "  ") > 0)
			{
				str = replaceAll(str, "  ", " ");
			}

			str = replaceAll(str.c_str(), "><", ">\n<");
			str = replaceAll(str.c_str(), "> <", ">\n<");

			char* token = strtok((char *)str.c_str(), "\n");
			while (token != NULL)
			{
				string str(token);
				if (
					(strstr(str.c_str(), "</p") > 0) ||
					(strstr(str.c_str(), "<br") > 0) ||
					((strstr(str.c_str(), "</h") > 0) && !(strstr(str.c_str(), "</head") > 0) && !(strstr(str.c_str(), "</html") > 0)) ||
					((strstr(str.c_str(), "</a") > 0) && (strstr(str.c_str(), "<a") > 0)) ||
					((strstr(str.c_str(), "</span") > 0) && (strstr(str.c_str(), "<span") > 0)) ||
					((strstr(str.c_str(), "<h") > 0) && !(strstr(str.c_str(), "<head") > 0) && !(strstr(str.c_str(), "<html") > 0)) ||
					((strstr(str.c_str(), "<img") > 0) && ((strstr(str.c_str(), ".jpg") > 0) || (strstr(str.c_str(), ".bmp") > 0)))
					)
				{
					parsedHTML.push_back(str);
					//wcout << endl;
				}
				token = strtok(NULL, "\n");
			}
		}
		return parsedHTML;
	}
	void ParsingURL(char* URL, char* addressBuffer, char* uRIBuffer)
	{
		char copiedURL[MAXLENGTH] = { 0, };
		strcpy(copiedURL, URL);

		int loopCount = 0;
		if (strstr(copiedURL, "//") != NULL)
		{
			loopCount = 1;
		}

		char* token = NULL;
		token = strtok(copiedURL, "/");
		for (int i = 0; i < loopCount; i++)
		{
			token = strtok(NULL, "/");
		}

		char addressChar[MAXLENGTH] = { 0, };
		strcpy(addressChar, token);
		strcpy(addressBuffer, addressChar);

		char uRI[MAXLENGTH] = { 0, };
		token = strtok(NULL, "");
		if (token != NULL)
		{
			strcpy(uRI, token);
		}
		else
		{
			strcpy(uRI, "/");
		}
		strcpy(uRIBuffer, uRI);
	}
	unsigned int FindCodePageIdentifier(vector<string> receivedHTML)
	{
		for (unsigned int i = 0; i < receivedHTML.size(); i++)
		{
			if (strstr(receivedHTML[i].c_str(), "utf-8") > 0)
			{
				return CP_UTF8;
			}
			else if (strstr(receivedHTML[i].c_str(), "euc-kr") > 0) // EUC Korean
			{
				return 51949; // EUC Korean
			}
		}
		return CP_ACP;
	}
};

class WebClient
{
private:
	SOCKET sockfd;
	WSADATA wsaData;
	struct sockaddr_in addr;
	void TranslateAddressToIpNumber(char* copiedAddress)
	{
		// translate address to ip number
		memset((void*)&addr, 0x00, sizeof(addr));
		addr.sin_family = AF_INET;

		struct hostent *remoteHost = gethostbyname(copiedAddress);
		struct in_addr inAddr = { 0 };
		if (remoteHost != NULL)
		{
			inAddr.s_addr = *(u_long *)remoteHost->h_addr_list[0];
			strcpy(copiedAddress, inet_ntoa(inAddr));
		}
	}
	void SetPortNumber(char* copiedAddress)
	{
		char* port;
		if (strstr(copiedAddress, ":") != NULL)
		{
			strtok(copiedAddress, ":");
			port = strtok(NULL, ":");
		}
		else
		{
			port = "80";
		}
		printf("port %s \n", port);
		printf("address %s \n", copiedAddress);
		addr.sin_addr.s_addr = inet_addr(copiedAddress);
		if (port != NULL)
		{
			addr.sin_port = htons(atoi(port));
		}
	}
	void ReceiveImageFile(string fileName)
	{
		char buf[MAXLENGTH] = { 0, };
		FILE* outFile = fopen(fileName.c_str(), "wb");
		int buffer = 0;


		//get some message and
		int receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
		while ((receiveCount > 0) && (outFile != NULL))
		{
			char* ptr1 = strpbrk(buf, "\r\n\r\n");
			if (ptr1 != NULL && strstr(ptr1, "\r\n\r\n") > 0)
			{
				// cutter
				int startIndex = (ptr1 - buf) + 4;
				receiveCount -= startIndex;

				// coper
				char copiedBuf[MAXLENGTH] = { 0, };
				for (int i = startIndex, j = 0; i < MAXLENGTH; i++, j++)
				{
					copiedBuf[j] = buf[i];
				}
				memset(buf, 0x00, MAXLENGTH);
				for (int i = 0; i < receiveCount; i++)
				{
					buf[i] = copiedBuf[i];
				}

				if (receiveCount == 0)
				{
					receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
				}

				// and rec rec rec
				while (receiveCount > 0)
				{
					fwrite(buf, 1, receiveCount, outFile);
					memset(buf, 0x00, MAXLENGTH);
					receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
				}
			}
			receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
		}
		if (outFile != NULL)
		{
			fclose(outFile);
		}
		closesocket(sockfd);
		WSACleanup();
	}
public:
	void ConnectServer(char* address)
	{
		char copiedAddress[MAXLENGTH] = { 0, };
		strcpy(copiedAddress, address);

		char buf[MAXLENGTH] = { 0, };

		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != NO_ERROR)
		{
			printf("error code 1\n");
		}
		if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		{
			printf("error code 2\n");
		}
		DWORD sock_timeout = 1 * 100; // for time out
		setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&sock_timeout, sizeof(sock_timeout));

		TranslateAddressToIpNumber(copiedAddress);
		SetPortNumber(copiedAddress);

		// set connection
		if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
		{
			printf("error code 3\n");
		}
		printf("im client, connect Job Done !\n");

	}
	void MakeAndSendMessage(char* uRI)
	{
		char tempPath[MAXLENGTH] = { 0, };
		tempPath[0] = '/';
		if (strstr(uRI, "/") == NULL)
		{
			strcat(tempPath, uRI);
		}
		else
		{
			strcpy(tempPath, uRI);
		}
		// make Send Message
		char buf[MAXLENGTH] = { 0, };
		memset(buf, 0x00, MAXLENGTH);
		strcpy(buf, "GET ");
		strcat(buf, tempPath);
		strcat(buf, " HTTP/1.1\r\n\r\n");
		send(sockfd, buf, strlen(buf), 0);
	}
	vector<string> GetReceiveHTMLMessage()
	{
		vector<string> receivedHTML;
		char buf[MAXLENGTH] = { 0, };
		int receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
		cout << buf << endl;
		for (int i = 0; i < STATUSCODELENGTH; i++)
		{
			if (strstr(buf, "200 OK") != NULL)
			{
				while (receiveCount > 0)
				{
					string receivedMessage(buf);
					receivedHTML.push_back(receivedMessage);
					memset(buf, 0x00, MAXLENGTH);
					receiveCount = recv(sockfd, buf, MAXLENGTH, 0);
				}
				closesocket(sockfd);//소캣 클로우즈
				WSACleanup();
				break;
			}
			else if (strstr(buf, statusCode[i].c_str()) != NULL) //예외처리
			{
				string tempString = "<h1>" + statusCode[i] + "</h1>";
				receivedHTML.push_back(tempString);
			}
		}
		return receivedHTML;
	}
	void RequestImages(vector<string> orderedHTMLSource, string address, string uRI)
	{
		Parser parser;
		for (unsigned int i = 0; i < orderedHTMLSource.size(); i++)
		{
			if (strstr(orderedHTMLSource[i].c_str(), "img src") != NULL)
			{
				string src = parser.SplitTag((char *)orderedHTMLSource[i].c_str(), '"', true, false);

				if (strstr(src.c_str(), "/") > 0)
				{
					char addBuffer[MAXLENGTH] = { 0, };
					char uriBuffer[MAXLENGTH] = { 0, };
					parser.ParsingURL((char *)src.c_str(), addBuffer, uriBuffer);
					string tempAdd(addBuffer);
					address = tempAdd;

					string tempURI(uriBuffer);
					uRI = tempURI;

					char* tok = strtok((char *)src.c_str(), "/");
					string beforeTok(tok);
					while (tok != NULL)
					{
						string temp(tok);
						beforeTok = temp;
						tok = strtok(NULL, "/");
					}
					src = beforeTok;
				}
				string imagePath(src);

				if (fopen(src.c_str(), "rb") == NULL)
				{
					//request image file
					ConnectServer((char *)address.c_str());
					if (imagePath.c_str()[0] == '.')
					{
						unsigned int i = 1;
						for (i = 1; i < imagePath.length(); i++)
						{
							imagePath[i - 1] = imagePath[i];
						}
						imagePath[i - 1] = '\0';
					}
					MakeAndSendMessage((char *)imagePath.c_str());
					ReceiveImageFile(src);
				}
			}
		}
	}
};

class Renderer
{
private:
	PAINTSTRUCT paintStructure;
	HDC handleDeviceContext;
	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	vector<string> orderedHTMLSource;
	int renderCoordinate = 50;

	void RenderJPGImage(string fileName, int ySpot, int height, int width)
	{
		GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

		WCHAR p[MAXLENGTH] = { 0, };

		for (unsigned int i = 0; i < fileName.length(); i++)
		{
			p[i] = fileName[i];
		}

		Image t(p);
		Graphics G(handleDeviceContext);

		if (G.GetLastStatus() != Ok)
		{
			printf("error\n");
		}
		if (height == -1 || width == -1)
		{
			height = t.GetHeight();
			width = t.GetWidth();
		}

		G.DrawImage(&t, 0, ySpot, width, height);
		renderCoordinate += height;
	}
	bool RenderBMPImage(string fileName, int ySpot, int height, int width)
	{
		if ((NULL == handleDeviceContext) || (NULL == fileName.c_str()))
		{
			return false;
		}
		wstring tempName = wstring(fileName.begin(), fileName.end());
		HANDLE hBmp = LoadImage(NULL, tempName.c_str(), IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);

		if (NULL == hBmp)
		{
			return false;
		}

		HDC dcmem = CreateCompatibleDC(NULL);

		if (NULL == SelectObject(dcmem, hBmp))
		{
			DeleteDC(dcmem);
			return false;
		}

		BITMAP bm;
		GetObject(hBmp, sizeof(bm), &bm);

		if (height == -1 || width == -1)
		{
			height = bm.bmHeight;
			width = bm.bmWidth;
		}

		if (BitBlt(handleDeviceContext, 0, ySpot, width, height, dcmem, 0, 0, SRCCOPY) == 0)
		{
			DeleteDC(dcmem);
			return false;
		}
		renderCoordinate += height;
		DeleteDC(dcmem);
		return true;
	}
	void RenderText(string text, int ySpot, int size)
	{
		string str = text;
		int length = MultiByteToWideChar(codePageIdentifier, 0, str.c_str(), -1, NULL, 0);
		wchar_t* output_buffer = new wchar_t[length];
		for (int i = 0; i < length; i++)
		{
			output_buffer[i] = L'\0';
		}

		MultiByteToWideChar(codePageIdentifier, 0, str.c_str(), -1, output_buffer, length);
		wstring wString(output_buffer);

		HFONT hFont = CreateFont(size, 0, 0, 0, 0, 0, 0, 0, HANGEUL_CHARSET, 0, 0, 0, VARIABLE_PITCH | FF_ROMAN, TEXT("맑은 고딕"));
		HFONT OldFont = (HFONT)SelectObject(handleDeviceContext, hFont);

		TextOutW(handleDeviceContext, 0, ySpot, wString.c_str(), wString.length());
		SelectObject(handleDeviceContext, OldFont);
		DeleteObject(hFont);
	}

public:
	Renderer()
	{
		xPos = 0;
		yPos = 0;
		xMax = 800;
		yMax = 800;
	}
	int yPos, xPos;
	int yMax, xMax;
	unsigned int codePageIdentifier = CP_ACP;
	bool isAlreadySetHyperLink = false;

	vector<string> movedHistory;
	vector<HWND> hyperTextWindowHandles;
	vector<string> hyperLinkTexts;
	int historyIndex = -1;
	void RenderContents()
	{
		handleDeviceContext = BeginPaint(mainWindowHandle, &paintStructure);

		renderCoordinate = (-100 * yPos);
		for (unsigned int i = 0; i < orderedHTMLSource.size(); i++)
		{

			if ((strstr(orderedHTMLSource[i].c_str(), "</h") != NULL) &&
				(orderedHTMLSource[i].length() > 5)
				)
			{
				int defaultSize = 5;
				char size = 0;
				if (strstr(orderedHTMLSource[i].c_str(), "<h") > 0)
				{
					strncpy(&size, &strstr(orderedHTMLSource[i].c_str(), "<h")[2], 1);
				}
				Parser parser;
				int hMinimumSize = 7;
				string tempItem = orderedHTMLSource[i];
				string outString = parser.SplitTag((char *)tempItem.c_str(), '<', false, true);
				if (renderCoordinate < 50 && renderCoordinate > -1)
				{
					renderCoordinate = 50;
				}
				RenderText(outString, renderCoordinate, defaultSize * (hMinimumSize - atoi(&size)));
				renderCoordinate += 20;
			}
			else if ((strstr(orderedHTMLSource[i].c_str(), "<a href=\"http://") > 0) &&
				(strstr(orderedHTMLSource[i].c_str(), ".html") > 0)
				)
			{
				if (renderCoordinate < 50 && renderCoordinate > -1)
				{
					renderCoordinate = 50;
				}
				if (isAlreadySetHyperLink == false)
				{
					RenderHyperText(orderedHTMLSource[i]);
				}
				else
				{
					/*
					RECT rect;
					GetWindowRect(mainWindowHandle, &rect);
					RedrawWindow(mainWindowHandle, &rect, NULL, RDW_ERASE);
					for (int i = 0; i < hyperTextWindowHandles.size(); i++)
					{
						RECT rect;
						GetWindowRect(hyperTextWindowHandles.at(i), &rect);
						InvalidateRect(hyperTextWindowHandles.at(i), &rect, false);
						cout << "number " << i << ", " << rect.left << " , " << rect.right << " , " << rect.right << " , " << rect.top << endl;
						MoveWindow(hyperTextWindowHandles.at(i), 0, renderCoordinate + (i * 40), 800, 35, false);
						UpdateWindow(hyperTextWindowHandles.at(i));
						//RedrawWindow(hyperTextWindowHandles.at(i), &rect, NULL, RDW_ERASE);
					}*/
				}
				renderCoordinate += 20;
			}
			else if (strstr(orderedHTMLSource[i].c_str(), "<img") != NULL)
			{
				renderCoordinate += 20;
				Parser parser;
				int height = -1, width = -1;
				char* token = NULL;
				char tempArray[MAXLENGTH] = { 0, };
				strcpy(tempArray, (char *)orderedHTMLSource[i].c_str());

				string src;
				token = strtok(tempArray, " ");
				while (token != NULL)
				{
					if (strstr(token, "height") > 0)
					{
						height = atoi(parser.SplitTag(token, '"', true, false).c_str());
					}
					else if (strstr(token, "width") > 0)
					{
						width = atoi(parser.SplitTag(token, '"', true, false).c_str());
					}
					else if (strstr(token, "src") > 0)
					{
						src = parser.SplitTag(token, '"', true, false);
					}
					token = strtok(NULL, " ");
				}
				if (strstr(orderedHTMLSource[i].c_str(), ".jpg") != NULL)
				{
					RenderJPGImage(src, renderCoordinate, height, width);
				}
				else if (strstr(orderedHTMLSource[i].c_str(), ".bmp") != NULL)
				{
					RenderBMPImage(src, renderCoordinate, height, width);
				}
			}
			else
			{
				char size = 1;
				Parser parser;
				int defaultSize = 5;
				int hMinimumSize = 7;
				if (renderCoordinate < 50 && renderCoordinate > -1)
				{
					renderCoordinate = 50;
				}
				renderCoordinate += 20;
				RenderText(parser.SplitTag((char *)orderedHTMLSource[i].c_str(), '<', false, true), renderCoordinate, 15);
			}
		}
		GdiplusShutdown(gdiplusToken);
		EndPaint(mainWindowHandle, &paintStructure);
		isAlreadySetHyperLink = true;
	}
	void SetOrderedHTMLSource(vector<string> inputMessage)
	{
		orderedHTMLSource = inputMessage;
	}
	void RenderHyperText(string inputString)
	{
		// Load and register SysLink control class
		INITCOMMONCONTROLSEX iccx;
		iccx.dwSize = sizeof(INITCOMMONCONTROLSEX);
		iccx.dwICC = ICC_LINK_CLASS;
		InitCommonControlsEx(&iccx);


		int length = MultiByteToWideChar(codePageIdentifier, 0, inputString.c_str(), -1, NULL, 0);
		wchar_t* output_buffer = new wchar_t[length];
		for (int i = 0; i < length; i++)
		{
			output_buffer[i] = L'\0';
		}

		MultiByteToWideChar(codePageIdentifier, 0, inputString.c_str(), -1, output_buffer, length);
		wstring wString(output_buffer);

		USES_CONVERSION;
		LPCTSTR sz = W2CT(wString.c_str());

		hLink = CreateWindowEx(0, L"SysLink",
			sz,//(LPCSTR)"<A HREF=\"http://www.microsoft.com\">click here</A> ",//(wString.c_str()),
			BS_GROUPBOX | WS_CHILD | WS_VISIBLE,
			0, renderCoordinate, 10 * inputString.length(), 35,
			mainWindowHandle, (HMENU)hyperTextWindowHandles.size(), hInst, 0);

		string tempString(wString.begin(), wString.end());
		string token = strtok((char *)tempString.c_str(), "\"");
		token = strtok(NULL, "\"");
		hyperLinkTexts.push_back(token);
		hyperTextWindowHandles.push_back(hLink);
		hyperLinkProc = (WNDPROC)SetWindowLongPtr(hLink, GWLP_WNDPROC, (LONG_PTR)ClickHyperLinkProcedure);
	}
};

Renderer renderer; // is single-ton value
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	// The main window class name.
	static TCHAR szWindowClass[] = _T("win32app");

	// The string that appears in the application's title bar.
	static TCHAR szTitle[] = _T("web browser");

	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = MainWindowProcedure;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_APPLICATION));
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = NULL;
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_APPLICATION));
	RegisterClassEx(&wcex);

	hInst = hInstance;
	mainWindowHandle = CreateWindow(szWindowClass, szTitle, WS_CLIPCHILDREN | WS_BORDER | WS_OVERLAPPEDWINDOW | WS_VSCROLL
		, CW_USEDEFAULT, CW_USEDEFAULT, 800, 800, NULL, NULL, hInstance, NULL);

	ShowWindow(mainWindowHandle, nCmdShow);
	UpdateWindow(mainWindowHandle);

	MSG message;
	while (GetMessage(&message, NULL, 0, 0))
	{
		TranslateMessage(&message);
		DispatchMessage(&message);
	}
	return (int)message.wParam;
}
LRESULT CALLBACK MainWindowProcedure(HWND handleWindow, UINT message, WPARAM wParam, LPARAM lParam)
{
	int yInc;
	GdiplusStartupInput gdiplusStartupInput;
	switch (message)
	{
	case WM_CREATE:
		CreateWindow(L"button", L"", WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 2, -6, 764, 41, handleWindow, (HMENU)3, hInst, NULL);
		backButtonWindowHandle = CreateWindow(L"button", L"←", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 5, 5, 25, 25, handleWindow, (HMENU)2, hInst, NULL);
		frontButtonWindowHandle = CreateWindow(L"button", L"→", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 32, 5, 25, 25, handleWindow, (HMENU)3, hInst, NULL);
		refleshButtonWindowHandle = CreateWindow(L"button", L"@", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 59, 5, 25, 25, handleWindow, (HMENU)4, hInst, NULL);


		textfieldWindowHandle = CreateWindow(L"EDIT", L"", WS_BORDER | WS_CHILD | WS_VISIBLE, 86, 5, 675, 25, handleWindow, (HMENU)1, NULL, NULL);
		SetFocus(handleWindow);
		OldEditProc = (WNDPROC)SetWindowLongPtr(textfieldWindowHandle, GWLP_WNDPROC, (LONG_PTR)URLTextFieldProcedure);
		break;
	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case 2: // go back
		{
			if (renderer.historyIndex > 0)
			{
				char uRL[MAXLENGTH] = { 0, };
				renderer.historyIndex--;
				strcpy(uRL, renderer.movedHistory.at(renderer.historyIndex).c_str());
				CommunicateServerToClient(uRL);
			}
			break;
		}
		case 3: // go front
		{
			if ((unsigned)renderer.historyIndex < renderer.movedHistory.size() - 1)
			{
				char uRL[MAXLENGTH] = { 0, };
				renderer.historyIndex++;
				strcpy(uRL, renderer.movedHistory.at(renderer.historyIndex).c_str());
				CommunicateServerToClient(uRL);
			}
			break;
		}
		case 4: // reflash
		{
			if (renderer.historyIndex > -1)
			{
				char uRL[MAXLENGTH] = { 0, };
				strcpy(uRL, renderer.movedHistory.at(renderer.historyIndex).c_str());
				CommunicateServerToClient(uRL);
			}
			break;
		}
		}
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	case WM_PAINT:
		renderer.RenderContents();
		break;
	case WM_VSCROLL:
		yInc = 0;
		switch (LOWORD(wParam))
		{
		case SB_LINEUP:
			yInc = -1;
			break;
		case SB_LINEDOWN:
			yInc = 1;
			break;
		case SB_PAGEUP:
			yInc = -1;
			break;
		case SB_PAGEDOWN:
			yInc = 1;
			break;
		case SB_THUMBTRACK:
			yInc = HIWORD(wParam) - renderer.yPos;
			break;
		}
		renderer.yPos = renderer.yPos + yInc;

		ScrollWindow(mainWindowHandle, 0, yInc*-100, (CONST RECT *) NULL, (CONST RECT *) NULL);
		SetScrollPos(mainWindowHandle, SB_VERT, renderer.yPos , true);
		cout << "renderer" << renderer.yPos << endl;

		InvalidateRect(handleWindow, NULL, TRUE);
		break;
	default:
		return DefWindowProc(handleWindow, message, wParam, lParam);
		break;
	}
	return 0;
}
LRESULT CALLBACK URLTextFieldProcedure(HWND handleWindow, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_KEYDOWN:
		if (wParam == VK_RETURN)
		{
			WCHAR textField[MAXLENGTH] = { 0, };

			GetWindowTextW(handleWindow, textField, MAXLENGTH);

			if ((unsigned)renderer.historyIndex < renderer.movedHistory.size() - 1)
			{
				while ((unsigned)renderer.historyIndex < renderer.movedHistory.size() - 1)
				{
					renderer.movedHistory.pop_back();
				}
			}
			char tempTextBuffer[MAXLENGTH] = { 0, };
			wcstombs(tempTextBuffer, textField, MAXLENGTH);
			string tempString(tempTextBuffer);

			renderer.historyIndex++;
			renderer.movedHistory.push_back(tempString.c_str());

			if (textField[0] != NULL)
			{
				CommunicateServerToClient((char *)tempString.c_str());
			}
		}
		break;
	case WM_PAINT:
		break;
	case WM_COMMAND:
		break;
	}
	return CallWindowProc(OldEditProc, handleWindow, message, wParam, lParam);
}
LRESULT CALLBACK ClickHyperLinkProcedure(HWND handleWindow, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_PAINT:
		break;
	case WM_LBUTTONDOWN:
		int tempNumber = LOWORD(wParam);
		for (unsigned int i = 0; i < renderer.hyperTextWindowHandles.size(); i++)
		{
			if (handleWindow == renderer.hyperTextWindowHandles[i])
			{
				renderer.historyIndex++;
				renderer.movedHistory.push_back(renderer.hyperLinkTexts[i].c_str());
				CommunicateServerToClient((char *)renderer.hyperLinkTexts[i].c_str());

				break;
			}
		}
		break;
	}
	return CallWindowProc(hyperLinkProc, handleWindow, message, wParam, lParam);
}

string replaceAll(const string &str, const string &pattern, const string &replace)
{
	string result = str;
	string::size_type pos = 0;
	string::size_type offset = 0;

	while ((pos = result.find(pattern, offset)) != string::npos)
	{
		result.replace(result.begin() + pos, result.begin() + pos + pattern.size(), replace);
		offset = pos + replace.size();
	}

	return result;
}
void CommunicateServerToClient(char* textField)
{
	char address[MAXLENGTH] = { 0, };
	char uRI[MAXLENGTH] = { 0, };
	WebClient client;
	Parser parser;
	vector<string> receivedStringVector;
	vector<string> tempStringVector;

	parser.ParsingURL(textField, address, uRI);
	client.ConnectServer(address);
	client.MakeAndSendMessage(uRI);
	string order(textField);

	int removeCount = renderer.hyperTextWindowHandles.size();
	for (int i = 0; i < removeCount; i++)
	{
		DestroyWindow(renderer.hyperTextWindowHandles.at(renderer.hyperTextWindowHandles.size() - 1));
		renderer.hyperTextWindowHandles.pop_back();
		renderer.hyperLinkTexts.pop_back();
	}

	receivedStringVector = client.GetReceiveHTMLMessage();
	renderer.codePageIdentifier = parser.FindCodePageIdentifier(receivedStringVector);
	tempStringVector = parser.GetParsedHTML(receivedStringVector);

	if (tempStringVector.size() == 0)
	{
		renderer.codePageIdentifier = 51949;
		tempStringVector.push_back("사랑합니다 고객님 !");
		tempStringVector.push_back("이 브라우져 정말 안 좋네요! 사이트를 보여줄 수 없다니!");
	}

	renderer.SetOrderedHTMLSource(tempStringVector);
	client.RequestImages(tempStringVector, address, uRI);
	renderer.isAlreadySetHyperLink = false;
	InvalidateRect(mainWindowHandle, NULL, TRUE);
}
