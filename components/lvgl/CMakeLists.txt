file(GLOB_RECURSE LVGL_SOURCES
	"src/*.c"
)

idf_component_register(
	SRCS ${LVGL_SOURCES}
	INCLUDE_DIRS
		"."
		"src"
		"src/lv_core"
		"src/lv_draw"
		"src/lv_font"
		"src/lv_hal"
		"src/lv_misc"
		"src/lv_themes"
)
