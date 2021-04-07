import http.server
import socketserver

serverPort = 8010
Handler = http.server.SimpleHTTPRequestHandler

if __name__ == "__main__":
    webServer = socketserver.TCPServer(("", serverPort), Handler)
    print("Server started at port %s" % (serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

        print("stop")
    webServer.server_close()
    print("Server stopped.")
