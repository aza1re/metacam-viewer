import React, { useEffect, useState } from "react";
import './App.css';

const isElectron = Boolean(window && window.electronAPI);

function App() {
  const [loading, setLoading] = useState(false);
  const [progress, setProgress] = useState({ step: 0, message: "" });
  const [outputDirs, setOutputDirs] = useState([]);
  const [selectedDir, setSelectedDir] = useState("");
  const [rerunOk, setRerunOk] = useState(null);
  const [pointcloudFiles, setPointcloudFiles] = useState([]);
  const [selectedPCD, setSelectedPCD] = useState("");

  useEffect(() => {
    if (isElectron && window.electronAPI && window.electronAPI.onProgressUpdate) {
      const removeListener = window.electronAPI.onProgressUpdate((event, data) => {
        setProgress(data);
      });
      return () => removeListener && removeListener();
    } else if (isElectron && window.require) {
      // Fallback for direct Electron ipcRenderer
      const { ipcRenderer } = window.require('electron');
      const handler = (event, data) => setProgress(data);
      ipcRenderer.on('progress-update', handler);
      return () => ipcRenderer.removeListener('progress-update', handler);
    }
  }, []);

  useEffect(() => {
    if (window.electronAPI && window.electronAPI.invoke) {
      window.electronAPI.invoke('check-rerun').then(setRerunOk);
    }
  }, []);

  const fetchOutputDirs = async () => {
    if (isElectron && window.electronAPI && window.electronAPI.listOutputDirs) {
      const dirs = await window.electronAPI.listOutputDirs();
      setOutputDirs(dirs);
      if (dirs.length > 0) setSelectedDir(dirs[0]);
    }
  };

  // Fetch pointcloud files after processing
  const fetchPointcloudFiles = async () => {
    if (isElectron && window.electronAPI && window.electronAPI.listPointcloudFiles) {
      const files = await window.electronAPI.listPointcloudFiles();
      setPointcloudFiles(files);
      if (files.length > 0) setSelectedPCD(files[0]);
    }
  };

  // Call fetchPointcloudFiles after processing is done
  useEffect(() => {
    if (progress.step === 4) fetchPointcloudFiles();
  }, [progress.step]);

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
            <button
              type="button"
              style={{ marginTop: 12, minWidth: 180 }}
              disabled={loading}
              onClick={async () => {
                setLoading(true);
                setProgress({ step: 0, message: "Starting..." });
                try {
                  await window.electronAPI.invoke('process-zip-and-view');
                  setProgress({ step: 4, message: "Done! 3D Viewer launched." });
                  await fetchOutputDirs();
                } catch (e) {
                  setProgress({ step: 0, message: "Error: " + e });
                  alert('Pipeline error: ' + e);
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
                  Processing...
                  <style>{`
                    @keyframes spin {
                      0% { transform: rotate(0deg); }
                      100% { transform: rotate(360deg); }
                    }
                  `}</style>
                </span>
              ) : (
                'One Click: Select Zip & View 3D'
              )}
            </button>
            {progress.message && (
              <div style={{ margin: '1rem 0', color: '#00447c', fontWeight: 500, textAlign: 'center' }}>
                {progress.message}
                {progress.message.includes('3D Viewer launched') && (
                  <div style={{ fontSize: 13, color: '#888', marginTop: 4 }}>
                    The 3D viewer will open in a separate window. If you don't see it, check your taskbar or desktop.<br/>
                    If you are running in a headless or remote environment, rerun.io may not be able to launch.
                  </div>
                )}
              </div>
            )}
            {outputDirs.length > 0 && (
              <div style={{ margin: '1rem 0', textAlign: 'center' }}>
                <label>
                  <b>Select output to view:</b>
                  <select
                    value={selectedDir}
                    onChange={e => setSelectedDir(e.target.value)}
                    style={{ marginLeft: 8, minWidth: 200 }}
                  >
                    {outputDirs.map(dir =>
                      <option key={dir} value={dir}>{dir.split('/').pop()}</option>
                    )}
                  </select>
                </label>
                <button
                  type="button"
                  style={{ marginLeft: 12 }}
                  disabled={!selectedDir}
                  onClick={async () => {
                    setProgress({ step: 5, message: `Launching viewer for ${selectedDir}` });
                    try {
                      await window.electronAPI.runViewer(selectedDir);
                      setProgress({ step: 6, message: "Done! 3D Viewer launched." });
                    } catch (e) {
                      setProgress({ step: 0, message: "Error: " + e });
                      alert('Viewer error: ' + e);
                    }
                  }}
                >View Selected Output</button>
                {/* New button for viewing with main.py */}
                <button
                  type="button"
                  style={{ marginLeft: 12, background: "#00447c", color: "#fff" }}
                  disabled={!selectedDir}
                  onClick={async () => {
                    setProgress({ step: 5, message: `Launching 3D viewer for ${selectedDir}` });
                    try {
                      await window.electronAPI.runViewer(selectedDir);
                      setProgress({ step: 6, message: "Done! 3D Viewer launched." });
                    } catch (e) {
                      setProgress({ step: 0, message: "Error: " + e });
                      alert('Viewer error: ' + e);
                    }
                  }}
                >View 3D Output</button>
                <button
                  type="button"
                  style={{ marginLeft: 12, background: "#e67c73", color: "#fff" }}
                  disabled={!selectedDir}
                  onClick={async () => {
                    setProgress({ step: 5, message: `Relaunching 3D viewer for ${selectedDir}` });
                    try {
                      await window.electronAPI.runViewer(selectedDir);
                      setProgress({ step: 6, message: "Done! 3D Viewer launched." });
                    } catch (e) {
                      setProgress({ step: 0, message: "Error: " + e });
                      alert('Viewer error: ' + e);
                    }
                  }}
                >
                  Relaunch 3D Viewer
                </button>
              </div>
            )}
            {pointcloudFiles.length > 0 && (
              <div style={{ margin: '2rem 0', textAlign: 'center' }}>
                <label>
                  <b>Select point cloud:</b>
                  <select
                    value={selectedPCD}
                    onChange={e => setSelectedPCD(e.target.value)}
                    style={{ marginLeft: 8, minWidth: 200 }}
                  >
                    {pointcloudFiles.map(f =>
                      <option key={f} value={f}>{f.split('/').pop()}</option>
                    )}
                  </select>
                </label>
                <button
                  type="button"
                  style={{ marginLeft: 12 }}
                  disabled={!selectedPCD}
                  onClick={async () => {
                    setProgress({ step: 5, message: `Launching viewer for ${selectedPCD}` });
                    try {
                      await window.electronAPI.runViewerPointcloud(selectedPCD);
                      setProgress({ step: 6, message: "Done! 3D Viewer launched." });
                    } catch (e) {
                      setProgress({ step: 0, message: "Error: " + e });
                      alert('Viewer error: ' + e);
                    }
                  }}
                >View Selected Point Cloud</button>
                <button
                  type="button"
                  style={{ marginLeft: 12, background: "#00447c", color: "#fff" }}
                  onClick={async () => {
                    setProgress({ step: 5, message: `Launching viewer for all point clouds` });
                    try {
                      await window.electronAPI.runViewerAllPointclouds();
                      setProgress({ step: 6, message: "Done! 3D Viewer launched." });
                    } catch (e) {
                      setProgress({ step: 0, message: "Error: " + e });
                      alert('Viewer error: ' + e);
                    }
                  }}
                >View All Point Clouds</button>
              </div>
            )}
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
        {rerunOk === false && (
          <div style={{color: 'red', fontWeight: 600, margin: 12}}>
            Python package <code>rerun</code> is <b>NOT</b> installed or importable!
          </div>
        )}
      </div>
    </div>
  );
}

export default App;

