import React, { useState } from "react";
import './App.css';

// Helper to detect Electron (robust: checks for window.electronAPI from preload)
const isElectron = Boolean(window && window.electronAPI);

function App() {
  const [folderPath, setFolderPath] = useState("");
  const [loading, setLoading] = useState(false);
  const [status, setStatus] = useState("");

  return (
    <div className="App">
      <header
        style={{
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          background: "#8cc64c",
          color: "#fff",
          padding: "1rem 2rem",
          borderRadius: "0 0 12px 12px",
          marginBottom: "2rem",
          boxShadow: "0 2px 8px rgba(0,0,0,0.08)",
        }}
      >
        <img
          src={process.env.PUBLIC_URL + "/hku-logo.png"}
          alt="HKU Logo"
          style={{ height: 48, marginRight: 20 }}
        />
        <h1 style={{ fontSize: "1.8rem", margin: 0, fontWeight: 600 }}>
          MetaCam Reader by INNOWING
        </h1>
      </header>
      <div style={{ maxWidth: 600, margin: "2rem auto", fontFamily: "sans-serif" }}>
        {isElectron ? (
          <form>
            <div>
              <button
                type="button"
                style={{ marginTop: 12 }}
                onClick={async () => {
                  const zipFile = await window.electronAPI.selectZip();
                  if (zipFile) {
                    setStatus('Unzipping data...');
                    setLoading(true);
                    try {
                      const folder = await window.electronAPI.unzipToTemp(zipFile);
                      setFolderPath(folder);
                      console.log(folder); // <-- Add this line to log the extracted folder path
                      setStatus('Ready to launch 3D Viewer.');
                    } catch (e) {
                      setStatus('Failed to unzip file.');
                      alert('Unzip error: ' + e);
                    }
                    setLoading(false);
                  }
                }}
              >
                Select Data Zip File
              </button>
              {folderPath && (
                <div style={{ fontSize: 12, color: '#555', marginTop: 4 }}>
                  Extracted to: {folderPath}
                </div>
              )}
            </div>
            <button
              type="button"
              disabled={loading || !folderPath}
              style={{ marginTop: 12 }}
              onClick={async () => {
                setLoading(true);
                setStatus("Launching 3D Viewer...");
                try {
                  const args = [
                    '3Dviewer/main.py',
                    folderPath
                  ];
                  await window.electronAPI.runPython(args);
                  setStatus("3D Viewer launched! (Check for a new window)");
                } catch (err) {
                  setStatus("");
                  alert('Python error: ' + err);
                }
                setLoading(false);
              }}
            >
              {loading ? (
                <span>
                  <span style={{
                    display: 'inline-block',
                    width: 18,
                    height: 18,
                    border: '3px solid #8cc64c',
                    borderTop: '3px solid #fff',
                    borderRadius: '50%',
                    animation: 'spin 1s linear infinite',
                    marginRight: 8,
                    verticalAlign: 'middle'
                  }} />
                  Launching 3D Viewer...
                  <style>{`
                    @keyframes spin {
                      0% { transform: rotate(0deg); }
                      100% { transform: rotate(360deg); }
                    }
                  `}</style>
                </span>
              ) : (
                'View 3D'
              )}
            </button>
          </form>
        ) : (
          <div style={{
            background: '#fffbe6',
            border: '1px solid #ffe58f',
            borderRadius: 8,
            padding: '2rem',
            textAlign: 'center',
            color: '#ad8b00',
            fontSize: 16,
            marginTop: '2rem'
          }}>
            <b>This app requires the MetaCam Reader Desktop (Electron) version.</b><br/>
            Please run <code>npm run electron</code> from your terminal to use the folder picker and 3D viewer.<br/>
            <br/>
            If you only see this message, you are running the web version (npm start), which does not support native features.
          </div>
        )}
        {status && !loading && isElectron && (
          <div style={{ margin: '1rem 0', color: '#00447c', fontWeight: 500, textAlign: 'center' }}>
            {status}
            {status.includes('3D Viewer launched') && (
              <div style={{ fontSize: 13, color: '#888', marginTop: 4 }}>
                The 3D viewer will open in a separate window. If you don't see it, check your taskbar or desktop.<br/>
                If you are running in a headless or remote environment, rerun.io may not be able to launch.
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

export default App;

