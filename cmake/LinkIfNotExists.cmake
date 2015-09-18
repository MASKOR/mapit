

MACRO(LINK_IF_DOES_NOT_EXISTS SOURCE DESTINATION)

#message(${PROJECT_NAME} ": Linking files: " ${SOURCE} " -> " ${DESTINATION})

        ADD_CUSTOM_COMMAND(
               TARGET ${PROJECT_NAME}
#               IF(NOT EXISTS ${DESTINATION})
#                   message("Linking files:" ${SOURCE} "->" ${DESTINATION})
                   COMMAND ln -sf ${SOURCE} ${DESTINATION}
#               else(NOT EXISTS ${DESTINATION})
#                   message("File already exists:" ${SOURCE})
#               ENDIF(NOT EXISTS ${DESTINATION})

        )


ENDMACRO(LINK_IF_DOES_NOT_EXISTS SOURCE DESTINATION)
