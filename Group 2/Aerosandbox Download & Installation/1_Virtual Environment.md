# Create an Isolated Virtual Environment

1. Open your terminal (**Command Prompt** or **PowerShell** on Windows, **Terminal** on macOS/Linux).

2. Navigate to your project folder:
   ```bash
   cd path/to/your/uav-propulsion-optimizer
   ```

3. Create the virtual environment (named `uav-env`):
   ```bash
   python -m venv uav-env
   ```

4. Activate the environment:
   - **Windows (Command Prompt):**
     ```cmd
     uav-env\Scripts\activate
     ```
   - **Windows (PowerShell):**
     ```powershell
     .\uav-env\Scripts\Activate.ps1
     ```
   - **macOS / Linux:**
     ```bash
     source uav-env/bin/activate
     ```

   > **Troubleshooting: "Running scripts is disabled on this system" (Windows PowerShell Only)**  
   > **Temporary Bypass (Highly Recommended):** Run this command in your active PowerShell terminal to lift the restriction for just your current session (it automatically restores default security when you close the terminal):
   > ```powershell
   > Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
   > ```
   > After running this, execute the activation script again:
   > ```powershell
   > .\uav-env\Scripts\Activate.ps1
   > ```

5. If done correctly, you will see `(uav-env)` at the left side of your terminal.