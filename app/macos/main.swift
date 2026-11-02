// AutoPathingFTC — native macOS wrapper for the offline web planner.
//
// Hosts app/web/index.html inside a WKWebView and exposes a small JS bridge
// ("autoPathBridge") so the planner can copy code, save files, and write
// generated OpModes straight into an FTC TeamCode folder.
//
// No paths are hardcoded. The TeamCode folder is resolved in this order:
//   1. AUTOPATHING_TEAMCODE_DIR environment variable (handy when launched from a terminal)
//   2. Folder previously chosen via "File ▸ Set TeamCode Folder…" (stored in UserDefaults)
//   3. Ask the user with a folder picker, then remember the choice

import Cocoa
import WebKit

private let kTeamCodeDirKey = "TeamCodeDirectory"
private let kBridgeName = "autoPathBridge"
private let kAppName = "AutoPathingFTC"

final class AppDelegate: NSObject, NSApplicationDelegate, WKScriptMessageHandler {
    var window: NSWindow!
    var webView: WKWebView!

    // MARK: - Lifecycle

    func applicationDidFinishLaunching(_ notification: Notification) {
        setupMenu()

        let screen = NSScreen.main?.visibleFrame ?? NSRect(x: 0, y: 0, width: 1280, height: 860)
        let width = min(1260, screen.width * 0.92)
        let height = min(860, screen.height * 0.92)
        let rect = NSRect(x: screen.origin.x + (screen.width - width) / 2,
                          y: screen.origin.y + (screen.height - height) / 2,
                          width: width, height: height)

        window = NSWindow(contentRect: rect,
                          styleMask: [.titled, .closable, .miniaturizable, .resizable],
                          backing: .buffered,
                          defer: false)
        window.title = "\(kAppName) — Visual Path Planner"
        window.appearance = NSAppearance(named: .darkAqua)

        let contentController = WKUserContentController()
        contentController.add(self, name: kBridgeName)
        let config = WKWebViewConfiguration()
        config.userContentController = contentController

        webView = WKWebView(frame: window.contentView!.bounds, configuration: config)
        webView.autoresizingMask = [.width, .height]
        window.contentView?.addSubview(webView)

        if let htmlURL = Bundle.main.url(forResource: "index", withExtension: "html") {
            webView.loadFileURL(htmlURL, allowingReadAccessTo: htmlURL.deletingLastPathComponent())
        }

        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    // MARK: - JS bridge

    func userContentController(_ userContentController: WKUserContentController,
                               didReceive message: WKScriptMessage) {
        guard let body = message.body as? [String: Any],
              let action = body["action"] as? String else { return }

        let code = body["code"] as? String ?? ""
        let fileName = sanitizedJavaFileName(body["fileName"] as? String)

        switch action {
        case "copy":
            NSPasteboard.general.clearContents()
            NSPasteboard.general.setString(code, forType: .string)

        case "saveDialog":
            let panel = NSSavePanel()
            panel.title = "Save FTC Java OpMode"
            panel.nameFieldStringValue = fileName
            panel.canCreateDirectories = true
            panel.directoryURL = FileManager.default.urls(for: .downloadsDirectory, in: .userDomainMask).first
            panel.beginSheetModal(for: window) { [weak self] response in
                guard response == .OK, let url = panel.url else { return }
                self?.write(code, to: url, successMessage: "✓ Saved \(url.lastPathComponent)")
            }

        case "sendToTeamCode":
            guard let dir = resolveTeamCodeDirectory() else {
                toast("TeamCode folder not set")
                return
            }
            let target = dir.appendingPathComponent(fileName)
            if FileManager.default.fileExists(atPath: target.path), !confirmOverwrite(fileName) {
                toast("Cancelled — \(fileName) left unchanged")
                return
            }
            write(code, to: target, successMessage: "✓ Sent \(fileName) to TeamCode")

        default:
            break
        }
    }

    // MARK: - Helpers

    /// Only allow a plain `Name.java` file name — no directories, no traversal.
    private func sanitizedJavaFileName(_ raw: String?) -> String {
        let base = (raw ?? "AutoPathOpMode.java")
            .replacingOccurrences(of: ".java", with: "")
            .filter { $0.isLetter || $0.isNumber || $0 == "_" }
        return (base.isEmpty ? "AutoPathOpMode" : base) + ".java"
    }

    private func resolveTeamCodeDirectory() -> URL? {
        if let env = ProcessInfo.processInfo.environment["AUTOPATHING_TEAMCODE_DIR"], !env.isEmpty {
            return URL(fileURLWithPath: (env as NSString).expandingTildeInPath, isDirectory: true)
        }
        if let saved = UserDefaults.standard.string(forKey: kTeamCodeDirKey),
           FileManager.default.fileExists(atPath: saved) {
            return URL(fileURLWithPath: saved, isDirectory: true)
        }
        return promptForTeamCodeDirectory()
    }

    @discardableResult
    private func promptForTeamCodeDirectory() -> URL? {
        let panel = NSOpenPanel()
        panel.title = "Choose your TeamCode folder"
        panel.message = "Select …/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
        panel.canChooseDirectories = true
        panel.canChooseFiles = false
        panel.allowsMultipleSelection = false
        guard panel.runModal() == .OK, let url = panel.url else { return nil }
        UserDefaults.standard.set(url.path, forKey: kTeamCodeDirKey)
        return url
    }

    private func confirmOverwrite(_ fileName: String) -> Bool {
        let alert = NSAlert()
        alert.messageText = "\(fileName) already exists"
        alert.informativeText = "Overwrite the existing OpMode in TeamCode? Tip: give each experiment a new class name so working versions are never lost."
        alert.alertStyle = .warning
        alert.addButton(withTitle: "Cancel")
        alert.addButton(withTitle: "Overwrite")
        return alert.runModal() == .alertSecondButtonReturn
    }

    private func write(_ code: String, to url: URL, successMessage: String) {
        do {
            try code.write(to: url, atomically: true, encoding: .utf8)
            toast(successMessage)
        } catch {
            toast("Error writing \(url.lastPathComponent)")
        }
    }

    private func toast(_ message: String) {
        let escaped = message
            .replacingOccurrences(of: "\\", with: "\\\\")
            .replacingOccurrences(of: "'", with: "\\'")
        webView.evaluateJavaScript("showToast('\(escaped)')", completionHandler: nil)
    }

    @objc private func setTeamCodeFolder(_ sender: Any?) {
        if let url = promptForTeamCodeDirectory() { toast("TeamCode folder: \(url.lastPathComponent)") }
    }

    // MARK: - Menu

    private func setupMenu() {
        let mainMenu = NSMenu()

        let appItem = NSMenuItem()
        let appMenu = NSMenu()
        appMenu.addItem(withTitle: "Quit \(kAppName)", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        appItem.submenu = appMenu
        mainMenu.addItem(appItem)

        let fileItem = NSMenuItem()
        let fileMenu = NSMenu(title: "File")
        let setDir = NSMenuItem(title: "Set TeamCode Folder…", action: #selector(setTeamCodeFolder(_:)), keyEquivalent: "t")
        setDir.target = self
        fileMenu.addItem(setDir)
        fileItem.submenu = fileMenu
        mainMenu.addItem(fileItem)

        // Edit menu enables ⌘C / ⌘V / ⌘A inside the web inputs
        let editItem = NSMenuItem()
        let editMenu = NSMenu(title: "Edit")
        editMenu.addItem(withTitle: "Undo", action: Selector(("undo:")), keyEquivalent: "z")
        editMenu.addItem(withTitle: "Redo", action: Selector(("redo:")), keyEquivalent: "Z")
        editMenu.addItem(.separator())
        editMenu.addItem(withTitle: "Cut", action: #selector(NSText.cut(_:)), keyEquivalent: "x")
        editMenu.addItem(withTitle: "Copy", action: #selector(NSText.copy(_:)), keyEquivalent: "c")
        editMenu.addItem(withTitle: "Paste", action: #selector(NSText.paste(_:)), keyEquivalent: "v")
        editMenu.addItem(withTitle: "Select All", action: #selector(NSText.selectAll(_:)), keyEquivalent: "a")
        editItem.submenu = editMenu
        mainMenu.addItem(editItem)

        NSApp.mainMenu = mainMenu
    }
}

let app = NSApplication.shared
app.setActivationPolicy(.regular)
let delegate = AppDelegate()
app.delegate = delegate
app.run()
