import AppKit
import WebKit

// Render the self-contained SVG with the same native CJK font fallback as a browser.
final class SVGRenderer: NSObject, WKNavigationDelegate {
    let view: WKWebView
    let output: URL
    let window: NSWindow

    init(output: URL) {
        self.output = output
        self.view = WKWebView(frame: NSRect(x: 0, y: 0, width: 1800, height: 1260))
        self.window = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 1800, height: 1260),
                               styleMask: [.borderless], backing: .buffered, defer: false)
        super.init()
        self.window.contentView = self.view
        self.view.navigationDelegate = self
    }

    func webView(_ webView: WKWebView, didFinish navigation: WKNavigation!) {
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.6) {
            let config = WKSnapshotConfiguration()
            config.rect = NSRect(x: 0, y: 0, width: 1800, height: 1260)
            config.snapshotWidth = 1800
            webView.takeSnapshot(with: config) { result, error in
                guard error == nil, let result = result, let tiff = result.tiffRepresentation,
                      let bitmap = NSBitmapImageRep(data: tiff),
                      let png = bitmap.representation(using: .png, properties: [:]) else {
                    fputs("SVG rendering failed: \(String(describing: error))\n", stderr)
                    exit(1)
                }
                do {
                    try png.write(to: self.output)
                    print("Rendered \(bitmap.pixelsWide) x \(bitmap.pixelsHigh): \(self.output.path)")
                    exit(0)
                } catch {
                    fputs("PNG write failed: \(error)\n", stderr)
                    exit(1)
                }
            }
        }
    }
}

guard CommandLine.arguments.count == 3 else {
    fputs("Usage: swift render_method_mainline_lark.swift input.svg output.png\n", stderr)
    exit(2)
}
let input = URL(fileURLWithPath: CommandLine.arguments[1])
let output = URL(fileURLWithPath: CommandLine.arguments[2])
let svg = try String(contentsOf: input, encoding: .utf8)
let application = NSApplication.shared
application.setActivationPolicy(.prohibited)
let renderer = SVGRenderer(output: output)
renderer.view.loadHTMLString("<!doctype html><html><head><meta charset=\"utf-8\"><style>html,body{margin:0;padding:0;width:1800px;height:1260px;overflow:hidden;background:white}svg{display:block}</style></head><body>\(svg)</body></html>", baseURL: input.deletingLastPathComponent())
application.run()
