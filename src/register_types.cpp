#include "register_types.h"
#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

//Singleton instance
static rclgd *_rclgd_singleton = nullptr;

// Modules initialization
void rclgd_init(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
	{
		return;
	}
	
	//Registration of ros runtime
	GDREGISTER_CLASS(rclgd)
	GDREGISTER_CLASS(RosNode)
	GDREGISTER_CLASS(RosPublisher)
	GDREGISTER_CLASS(RosSubscriber)
	GDREGISTER_CLASS(RosClient)
	GDREGISTER_CLASS(RosService)
	GDREGISTER_CLASS(RosMsg) //Instance Ros2 Type Creator


	//TF2 Helpers
	GDREGISTER_CLASS(RosNode3D)



	//Create the rclgd singleton
	_rclgd_singleton = memnew(rclgd);
    Engine::get_singleton()->register_singleton("rclgd", rclgd::get_singleton());
}

void rclgd_deinit(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
	{
		return;
	}
	// Remove the global name
    Engine::get_singleton()->unregister_singleton("rclgd");

    // Delete the instance (this triggers the destructor ~rclgd() where we join the thread)
    if (_rclgd_singleton) {
        memdelete(_rclgd_singleton);
    }
}

extern "C"
{
	// Initialization.
	GDExtensionBool GDE_EXPORT rclgd_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
	{
		godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

		init_obj.register_initializer(rclgd_init);
		init_obj.register_terminator(rclgd_deinit);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

		return init_obj.init();
	}
}
