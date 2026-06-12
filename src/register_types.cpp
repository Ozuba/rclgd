#include "register_types.h"
#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>

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
	//Publishers & Subscribers
	GDREGISTER_CLASS(RosPublisher)
	GDREGISTER_CLASS(RosSubscriber)
	//Services
	GDREGISTER_CLASS(RosClient)
	GDREGISTER_CLASS(RosRequest)
	GDREGISTER_CLASS(RosService)
	//Actions
	GDREGISTER_CLASS(RosActionClient)
	GDREGISTER_CLASS(RosGoalHandle)
	GDREGISTER_CLASS(RosActionServer)
	GDREGISTER_CLASS(RosServerGoalHandle)
	//TF2
	GDREGISTER_CLASS(RosTfBroadcaster)
	GDREGISTER_CLASS(RosTfListener)
	//Timer
	GDREGISTER_CLASS(RosTimer)

	GDREGISTER_CLASS(RosMsg) //Instance Ros2 Type Creator
	GDREGISTER_CLASS(RosQoS) //Instance Ros2 Type Creator


	// Register the project setting that controls whether RCLGD errors break
	// into the script debugger (see RCLGD_FAIL_* macros).
	ProjectSettings *ps = ProjectSettings::get_singleton();
	if (ps && !ps->has_setting("rclgd/debug/break_on_error"))
	{
		ps->set_setting("rclgd/debug/break_on_error", true);
		ps->set_initial_value("rclgd/debug/break_on_error", true);
	}

	//Create the rclgd singleton
	_rclgd_singleton = memnew(rclgd);
    Engine::get_singleton()->register_singleton("rclgd", rclgd::get_singleton());
}

void rclgd_deinit(ModuleInitializationLevel p_level)
{
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE)
	{
		// Remove the global name
		Engine::get_singleton()->unregister_singleton("rclgd");

		// Destroying the singleton joins the executor thread and shuts
		// rclcpp down (see rclgd::~rclgd), so the process can exit cleanly
		// even if the user never called rclgd.shutdown().
		if (_rclgd_singleton)
		{
			memdelete(_rclgd_singleton);
			_rclgd_singleton = nullptr;
		}
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
