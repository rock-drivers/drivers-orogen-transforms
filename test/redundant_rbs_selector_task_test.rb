# frozen_string_literal: true

using_task_library "transforms"

describe OroGen.transforms.RedundantRBSSelectorTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.transforms.RedundantRBSSelectorTask
                  .deployed_as("test_task")
        )

        @task.properties.source_timeout = Time.at(1)
        @task.properties.main_source_histeresys = Time.at(5)
        @task.properties.init_timeout = Time.at(1)
        @task.properties.position_threshold = 2
        @task.properties.orientation_thresholds = [0.5, 0.4, 0.3]

        syskit_configure(@task)
    end

    it "forwards main source when BOTH_SOURCES_VALID" do
        main_w = syskit_create_writer task.main_rbs_source_port
        secondary_w = syskit_create_writer task.secondary_rbs_source_port

        main_rbs = rbs({ data: [1, 0, 0] })
        secondary_rbs = rbs({ data: [-1, 0, 0] })

        out = expect_execution { task.start! }.poll do
            main_w.write main_rbs
            secondary_w.write secondary_rbs
        end.to do # rubocop:disable Style/MultilineBlockChain
            emit task.both_sources_valid_event
            custom_runtime_states(except: ["both_sources_valid"]).each do |state|
                not_emit task.send("#{state}_event")
            end
            have_new_samples(task.rbs_out_port, 20)
        end

        assert_equal([1] * 20, out.map { |v| v.position.x })
    end

    describe "pose comparation" do
        it "returns true when the position diference is bigger than the position "\
        "threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs = rbs({ data: [-5, 0, 0] })
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { position_divergent: 1, position_error_norm: 6 }
            assert_divergence(result, expected)
        end

        it "returns false when the position difference is smaller than the position "\
        "threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs = rbs({ data: [2, 0, 0] })
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { position_error_norm: 1 }
            assert_divergence(result, expected)
        end

        it "returns true when the yaw difference is bigger than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.4, Eigen::Vector3.UnitZ)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(0.2, Eigen::Vector3.UnitZ)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { yaw_divergent: 1, yaw_error: -0.6 }
            assert_divergence(result, expected)
        end

        it "returns false when the yaw difference is smaller than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.2, Eigen::Vector3.UnitZ)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitZ)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { yaw_error: -0.2 }
            assert_divergence(result, expected)
        end

        it "returns true when the pitch difference is bigger than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.5, Eigen::Vector3.UnitY)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitY)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { pitch_divergent: 1, pitch_error: -0.5 }
            assert_divergence(result, expected)
        end

        it "returns false when the pitch difference is smaller than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.3, Eigen::Vector3.UnitY)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitY)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { pitch_error: -0.3 }
            assert_divergence(result, expected)
        end

        it "returns true when the roll difference is bigger than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(0.1, Eigen::Vector3.UnitX)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.6, Eigen::Vector3.UnitX)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { roll_divergent: 1, roll_error: 0.7 }
            assert_divergence(result, expected)
        end

        it "returns false when the roll difference is smaller than its threshold" do
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            expect_execution { task.start! }.to { emit task.both_sources_valid_event }
            main_rbs = rbs({ data: [1, 0, 0] })
            main_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.1, Eigen::Vector3.UnitX)
            secondary_rbs = rbs({ data: [1, 0, 0] })
            secondary_rbs.orientation =
                Eigen::Quaternion.from_angle_axis(-0.3, Eigen::Vector3.UnitX)
            result = expect_execution.poll do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do
                have_one_new_sample(task.pose_divergence_port)
            end
            expected = { roll_error: 0.2 }
            assert_divergence(result, expected)
        end

        def assert_divergence(
            result, position_divergent: 0, roll_divergent: 0, pitch_divergent: 0,
            yaw_divergent: 0, position_error_norm: 0, roll_error: 0, pitch_error: 0,
            yaw_error: 0
        )
            assert_equal(result.position_divergent, position_divergent)
            assert_equal(result.roll_divergent, roll_divergent)
            assert_equal(result.pitch_divergent, pitch_divergent)
            assert_equal(result.yaw_divergent, yaw_divergent)
            assert_in_delta(result.position_error_norm, position_error_norm, 1e-3)
            assert_in_delta(result.roll_error.rad, roll_error, 1e-3)
            assert_in_delta(result.pitch_error.rad, pitch_error, 1e-3)
            assert_in_delta(result.yaw_error.rad, yaw_error, 1e-3)
        end
    end

    describe "validates rbs sources" do
        it "validates position" do
            rbs_s = rbs
            rbs_s.position = { data: [NaN] * 3 }
            assert_no_valid_sources rbs_s
        end

        it "validates orientation" do
            rbs_s = rbs
            rbs_s.orientation = { im: [NaN] * 3, re: NaN }
            assert_no_valid_sources rbs_s
        end

        it "validates linear velocity" do
            rbs_s = rbs
            rbs_s.velocity = { data: [NaN] * 3 }
            assert_no_valid_sources rbs_s
        end

        it "validates angular velocity" do
            rbs_s = rbs
            rbs_s.angular_velocity = { data: [NaN] * 3 }
            assert_no_valid_sources rbs_s
        end

        def assert_no_valid_sources(invalid_rbs)
            syskit_start(task)

            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port

            expect_execution.poll do
                main_w.write invalid_rbs
                secondary_w.write invalid_rbs
            end.to do
                emit task.no_valid_sources_event
                emit task.exception_event
            end
        end
    end

    describe "transitions from MAIN_SOURCE_RECOVERING" do
        it "transition from MAIN_SOURCE_RECOVERING to NO_VALID_SOURCES" do
            syskit_start(task)
            transition_to_main_source_recovering

            expect_execution.to do
                emit task.no_valid_sources_event
                custom_runtime_states(except: ["no_valid_sources"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        it "transitions from MAIN_SOURCE_RECOVERING to INVALID_MAIN_SOURCE" do
            syskit_start(task)

            _, secondary_w = transition_to_main_source_recovering

            rbs_s = rbs({ data: [1, 2, 3] })
            expect_execution.poll do
                secondary_w.write rbs_s
            end.to do # rubocop:disable Style/MultilineBlockChain
                task.invalid_main_source_event
                custom_runtime_states(except: ["invalid_main_source"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        it "transitions from MAIN_SOURCE_RECOVERING to INVALID_SECONDARY_SOURCE" do
            syskit_start(task)

            main_w, = transition_to_main_source_recovering

            rbs_s = rbs({ data: [4, 5, 6] })
            expect_execution.poll do
                main_w.write rbs_s
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.invalid_secondary_source_event
                custom_runtime_states(except: ["invalid_secondary_source"])
                    .each do |state|
                        not_emit task.send("#{state}_event")
                    end
            end
        end

        it "transitions from MAIN_SOURCE_RECOVERING to BOTH_SOURCES_VALID" do
            syskit_start(task)
            main_w, secondary_w = transition_to_main_source_recovering

            rbs_main = rbs({ data: [2, 4, 6] })
            rbs_secondary = rbs({ data: [4, 8, 12] })
            expect_execution.poll do
                main_w.write rbs_main
                secondary_w.write rbs_secondary
            end.to do
                custom_runtime_states(except: ["both_sources_valid"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
                emit task.both_sources_valid_event
            end
        end

        def transition_to_main_source_recovering
            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            secondary_rbs = rbs({ data: [-1, 0, 0] })
            expect_execution.poll do
                secondary_w.write secondary_rbs
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.invalid_main_source_event
            end

            main_rbs = rbs({ data: [1, 0, 0] })
            main_w = syskit_create_writer task.main_rbs_source_port

            expect_execution do
                main_w.write main_rbs
                secondary_w.write secondary_rbs
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.main_source_recovering_event
            end

            [main_w, secondary_w]
        end
    end

    describe "transitions from BOTH_SOURCES_VALID" do
        it "transitions to NO_VALID_SOURCES when port driven timeout" do
            expect_execution { task.start! }
                .to do
                    not_emit task.both_sources_valid_event
                    emit task.no_valid_sources_event
                    emit task.exception_event
                end
        end

        it "transitions from BOTH_SOURCES_VALID to INVALID_MAIN_SOURCE, " \
           "when just secondary is available" do
            syskit_start(task)

            secondary_w = syskit_create_writer task.secondary_rbs_source_port
            rbs_s = rbs({ data: [0, 0, 0] })
            expect_execution.poll do
                secondary_w.write rbs_s
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.invalid_main_source_event
                custom_runtime_states(except: ["invalid_main_source"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end
        it "transitions from BOTH_SOURCES_VALID to INVALID_SECONDARY_SOURCE, " \
           "when just secondary is available" do
            syskit_start(task)

            main_w = syskit_create_writer task.main_rbs_source_port
            rbs_s = rbs({ data: [0, 0, 0] })
            expect_execution.poll do
                main_w.write rbs_s
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.invalid_secondary_source_event
                custom_runtime_states(except: ["invalid_secondary_source"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end
    end

    describe "transitions from INVALID_MAIN_SOURCE" do
        it "transitions from INVALID_MAIN_SOURCE to NO_VALID_SOURCES" do
            syskit_start(task)
            transition_to_invalid_main_source
            expect_execution.to do
                emit task.no_valid_sources_event
                custom_runtime_states(except: ["no_valid_sources"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        it "transitions from INVALID_MAIN_SOURCE to MAIN_SOURCE_RECOVERING" do
            syskit_start(task)
            main_w, secondary_w = transition_to_invalid_main_source
            rbs_main = rbs({ data: [0, 1, 2] })
            rbs_secondary = rbs({ data: [0, 2, 4] })
            expect_execution.poll do
                main_w.write rbs_main
                secondary_w.write rbs_secondary
            end.to do
                emit task.main_source_recovering_event
                custom_runtime_states(except: ["main_source_recovering"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        def transition_to_invalid_main_source
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port

            rbs_s = rbs({ data: [1, 2, 3] })
            expect_execution.poll do
                secondary_w.write rbs_s
            end.to do
                emit task.invalid_main_source_event
            end

            [main_w, secondary_w]
        end
    end

    describe "transitions from INVALID_SECONDARY_SOURCE" do
        it "transitions from INVALID_SECONDARY_SOURCE to BOTH_SOURCES_VALID" do
            syskit_start(task)

            main_w, secondary_w = transition_to_invalid_secondary_source

            rbs_main = rbs({ data: [1, 2, 3] })
            rbs_secondary = rbs({ data: [2, 4, 6] })

            expect_execution.poll do
                main_w.write rbs_main
                secondary_w.write rbs_secondary
            end.to do
                emit task.both_sources_valid_event
                custom_runtime_states(except: ["both_sources_valid"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        it "transitions from INVALID_SECONDARY_SOURCE to NO_VALID_SOURCES" do
            syskit_start(task)
            transition_to_invalid_secondary_source
            expect_execution.to do
                emit task.no_valid_sources_event
                custom_runtime_states(except: ["no_valid_sources"]).each do |state|
                    not_emit task.send("#{state}_event")
                end
            end
        end

        def transition_to_invalid_secondary_source
            main_w = syskit_create_writer task.main_rbs_source_port
            secondary_w = syskit_create_writer task.secondary_rbs_source_port

            rbs_s = rbs({ data: [1, 2, 3] })
            expect_execution.poll do
                main_w.write rbs_s
            end.to do # rubocop:disable Style/MultilineBlockChain
                emit task.invalid_secondary_source_event
            end

            [main_w, secondary_w]
        end
    end

    def custom_runtime_states(except: [])
        %w[
            main_source_recovering
            invalid_main_source
            invalid_secondary_source
            no_valid_sources
            both_sources_valid
        ] - except
    end

    def rbs(position = { data: [0] * 3 })
        cov3 = { data: [NaN] * 9 }
        Types.base.samples.RigidBodyState.new(
            time: Time.now,
            position: position,
            cov_position: cov3,
            orientation: { im: [0] * 3, re: 1 },
            cov_orientation: cov3,
            velocity: { data: [0] * 3 },
            cov_velocity: cov3,
            angular_velocity: { data: [0] * 3 },
            cov_angular_velocity: cov3
        )
    end
end