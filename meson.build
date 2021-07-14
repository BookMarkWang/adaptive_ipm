project('adaptive_ipm', 'cpp')

cc = meson.get_compiler('cpp')

opencv_dep= dependency('opencv4')
pthread_dep= cc.find_library('pthread')
m_dep = cc.find_library('m', required : false)

src = files('camera_parameters.cpp', 'adaptive_ipm.cpp', 'main.cpp')

executable('adaptive_ipm', src,
            dependencies : [m_dep, opencv_dep, pthread_dep],
            cpp_args : ['-std=c++17','-O2'])

