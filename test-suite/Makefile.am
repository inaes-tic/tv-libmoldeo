INCLUDES                    = @CHECK_CFLAGS@ -I../libmoldeo
TESTS                       = check_libmoldeo
noinst_PROGRAMS             = $(TESTS)
check_libmoldeo_SOURCES     = check_libmoldeo.cpp
check_libmoldeo_CPPFLAGS    = ${FT2_CFLAGS} \
	${GST_CFLAGS} \
	${MUPARSER_CFLAGS} \
	${BOOST_CPPFLAGS} \
	${SDL_CFLAGS} \
	${LUA51_CFLAGS} 
check_libmoldeo_LIBS    = ${FT2_LIBS} \
	${GST_LIBS} \
	${BOOST_ASIO_LIB} \
	${BOOST_THREAD_LIB} \
	${BOOST_FILESYSTEM_LIB} \
	${LUA51_LIBS} 
check_libmoldeo_LDADD       = @CHECK_LIBS@ \
	-L../libmoldeo -lmoldeo \
	@MUPARSER_LIBS@ \
	@SDL_LIBS@ \
	@GLE_LIBS@ \
	@FREEIMAGE_LIBS@ \
	@FREEALUT_LIBS@ \
	@GLEW_LIBS@ \
	@CG_LIBS@ \
	@CGGL_LIBS@ \
	@LUA51_LIBS@ \
	@GST_LIBS@ \
	@TINYXML_LIBS@ \
	@FT2_LIBS@ \
	@BOOST_ASIO_LIB@ \
	@BOOST_FILESYSTEM_LIB@ \
	@BOOST_THREAD_LIB@ \
	@FTGL_LIBS@ \
	${NULL}

CLEANFILES                  = check_libmoldeo.log


