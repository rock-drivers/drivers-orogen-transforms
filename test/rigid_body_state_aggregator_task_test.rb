# frozen_string_literal: true

using_task_library "transforms"

describe OroGen.transforms.RigidBodyStateAggregatorTask do
    attr_reader :task
    before do
        @task_m = OroGen.transforms.RigidBodyStateAggregatorTask
                        .deployed_as("test_task")
        @task = syskit_deploy(@task_m)
    end

    it "forwards a sample" do
        sample = Types.base.samples.RigidBodyState.Invalid
        sample.time = Time.at(1)

        syskit_configure_and_start(task)
        expect_execution { syskit_write task.source2ref_port, sample }
            .to do
                have_one_new_sample(task.source2ref_aggregated_port)
                    .matching { |s| s.time == Time.at(1) }
            end
    end

    it "ignores samples older than most recent outputed sample" do
        sample = Types.base.samples.RigidBodyState.Invalid
        sample.time = Time.at(2)

        syskit_configure_and_start(task)

        w = syskit_create_writer task.source2ref_port
        samples = expect_execution do
            w.write sample
            w.write sample.tap { |rbs| rbs.time = Time.at(1) }
            w.write sample.tap { |rbs| rbs.time = Time.at(3) }
        end.to { have_new_samples(task.source2ref_aggregated_port, 2) } # rubocop:disable Style/MultilineBlockChain

        assert_equal Time.at(2), samples.first.time
        assert_equal Time.at(3), samples.last.time
    end
end
