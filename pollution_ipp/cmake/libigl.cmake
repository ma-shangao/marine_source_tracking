if(TARGET igl::core)
    return()
endif()

include(FetchContent)
fetchcontent_declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.6.0
)
fetchcontent_makeavailable(libigl)
