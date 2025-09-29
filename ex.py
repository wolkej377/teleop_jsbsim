import jsbsim
fdm = jsbsim.FGFDMExec(root_dir=None)
fdm.load_model("c310")
fdm.print_property_catalog()
