# Install AeroSandbox

To ensure pip downloads pre-compiled binaries instead of attempting to compile complex packages (like CasADi, SciPy, or Matplotlib) from source, upgrade your core installation tools first, then run the installer with the binary override flag:

1. **Upgrade your pip package installer:**
   ```bash
   python -m pip install --upgrade pip setuptools wheel
   ```

2. **Install AeroSandbox and its dependencies using pre-compiled binaries only:**
   ```bash
   pip install aerosandbox matplotlib scipy numpy --only-binary=:all:
   ```

3. **Verification:**
   - Confirm everything is installed correctly by creating a verification script named `test_asb.py`:
     ```python
     import aerosandbox as asb
     import numpy as np

     print("--- AeroSandbox Installation Verification ---")
     try:
         # Build a quick test airfoil to make sure the math libraries are working
         naca4412 = asb.Airfoil("naca4412")
         print(f"Airfoil successfully loaded: {naca4412.name}")
         print("AeroSandbox is 100% operational!")
     except Exception as e:
         print(f"Verification failed with error: {e}")
     ```

   - Run the script in your active terminal:
     ```bash
     python test_asb.py
     ```

   - If you see `AeroSandbox is 100% operational!` printed to your screen, your environment is ready to start optimizing custom propellers!