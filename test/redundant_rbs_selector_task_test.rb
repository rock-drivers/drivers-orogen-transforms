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

        syskit_configure(@task)
    end

    it "forwards main source when BOTH_SOURCES_VALID" do
        expect_execution { task.start! }.to { emit task.both_sources_valid_event }

        main_w = syskit_create_writer task.main_rbs_source_port
        secondary_w = syskit_create_writer task.secondary_rbs_source_port

        main_rbs = rbs({ data: [1, 0, 0] })
        secondary_rbs = rbs({ data: [-1, 0, 0] })
        out = expect_execution.poll do
            main_w.write main_rbs
            secondary_w.write secondary_rbs
        end.to do # rubocop:disable Style/MultilineBlockChain
            custom_runtime_states.each do |state|
                not_emit task.send("#{state}_event")
            end
            have_new_samples(task.rbs_out_port, 20)
        end

        out.each do |s|
            assert_equal 1, s.position[0]
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
            syskit_start(task)

            expect_execution.to do
                emit task.no_valid_sources_event
                emit task.exception_event
            end
        end

        it "transitions from BOTH_SOURCES_VALID to INVALID_MAIN_SOURCE, \
    when just secondary is available" do
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
        it "transitions from BOTH_SOURCES_VALID to INVALID_SECONDARY_SOURCE, \
    when just secondary is available" do
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

            rbs_s = rbs( { data: [1, 2, 3] })
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

    def rbs(position)
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