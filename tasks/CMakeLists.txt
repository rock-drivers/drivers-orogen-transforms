# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(transformsTaskLib)
ADD_LIBRARY(${TRANSFORMS_TASKLIB_NAME} SHARED
    ${TRANSFORMS_TASKLIB_SOURCES})
add_dependencies(${TRANSFORMS_TASKLIB_NAME}
    regen-typekit)


TARGET_LINK_LIBRARIES(${TRANSFORMS_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${TRANSFORMS_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${TRANSFORMS_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${TRANSFORMS_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${TRANSFORMS_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${TRANSFORMS_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${TRANSFORMS_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${TRANSFORMS_TASKLIB_HEADERS}
    DESTINATION include/orocos/transforms)

