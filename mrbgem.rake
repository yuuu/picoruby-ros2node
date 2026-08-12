MRuby::Gem::Specification.new('picoruby-ros2node') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Yuhei Okazaki'
  spec.summary = 'Minimal ROS2 node via Micro-XRCE-DDS-Client'

  def ensure_vendored_lib!(lib_dir, version, repo)
    if File.directory?("#{lib_dir}/.git")
      current_tag = `cd #{lib_dir} && git describe --tags --exact-match HEAD 2>/dev/null`.strip
      unless current_tag == version
        puts "#{File.basename(lib_dir)} version mismatch. Fetching and checking out #{version}..."
        sh "cd #{lib_dir} && git fetch origin #{version}:#{version} 2>/dev/null || git fetch origin"
        sh "cd #{lib_dir} && git checkout #{version}"
      end
    elsif File.directory?(lib_dir)
      puts "#{File.basename(lib_dir)} directory exists but is not a git repository. Removing and cloning..."
      FileUtils.rm_rf(lib_dir)
      sh "git clone -b #{version} #{repo} #{lib_dir}"
    else
      sh "git clone -b #{version} #{repo} #{lib_dir}"
    end
  end

  def symlink!(link_path, target_path)
    FileUtils.mkdir_p(File.dirname(link_path))
    FileUtils.ln_sf(target_path, link_path)
  end

  def add_vendored_sources!(lib_dir, rel_paths)
    rel_paths.each do |rel_path|
      src = "#{lib_dir}/#{rel_path}"
      obj = "#{build_dir}/#{rel_path.pathmap('%X')}.o"
      file obj => src do |t|
        cc.run t.name, t.prerequisites[0]
      end
      objs << obj
    end
  end

  uxrcedds_dir = "#{dir}/lib/Micro-XRCE-DDS-Client"
  microcdr_dir = "#{dir}/lib/MicroCDR"

  ensure_vendored_lib!(uxrcedds_dir, "v3.0.1", "https://github.com/eProsima/Micro-XRCE-DDS-Client.git")
  ensure_vendored_lib!(microcdr_dir, "v2.0.2", "https://github.com/eProsima/Micro-CDR.git")

  # These libraries ship include/uxr/client/config.h.in / include/ucdr/config.h.in,
  # normally rendered by CMake's configure_file(). We hand-write the rendered
  # result and symlink it into place instead.
  symlink!("#{uxrcedds_dir}/include/uxr/client/config.h", "#{dir}/include/ros2node/uxr_config.h")
  symlink!("#{microcdr_dir}/include/ucdr/config.h", "#{dir}/include/ros2node/ucdr_config.h")

  spec.cc.include_paths << "#{uxrcedds_dir}/include"
  spec.cc.include_paths << "#{uxrcedds_dir}/src/c"
  spec.cc.include_paths << "#{microcdr_dir}/include"

  add_vendored_sources!(microcdr_dir, %w[
    src/c/common.c
    src/c/types/basic.c
    src/c/types/string.c
    src/c/types/array.c
    src/c/types/sequence.c
  ])

  add_vendored_sources!(uxrcedds_dir, %w[
    src/c/core/session/stream/input_best_effort_stream.c
    src/c/core/session/stream/input_reliable_stream.c
    src/c/core/session/stream/output_best_effort_stream.c
    src/c/core/session/stream/output_reliable_stream.c
    src/c/core/session/stream/stream_storage.c
    src/c/core/session/stream/stream_id.c
    src/c/core/session/stream/seq_num.c
    src/c/core/session/session.c
    src/c/core/session/session_info.c
    src/c/core/session/submessage.c
    src/c/core/session/object_id.c
    src/c/core/serialization/xrce_types.c
    src/c/core/serialization/xrce_header.c
    src/c/core/serialization/xrce_subheader.c
    src/c/util/time.c
    src/c/util/ping.c
    src/c/core/session/common_create_entities.c
    src/c/core/session/create_entities_ref.c
    src/c/core/session/create_entities_xml.c
    src/c/core/session/create_entities_bin.c
    src/c/core/session/read_access.c
    src/c/core/session/write_access.c
    src/c/profile/transport/stream_framing/stream_framing_protocol.c
    src/c/profile/transport/custom/custom_transport.c
  ])
end
