

#      Some Example Usages
#
#      The following is for a PUT to the sites API:
#
#      response = ApiAdapter.send_command(
#        :resource => "/sites/6af934acd83f",
#        :method => :put,
#        :payload => {"data_post" => "(site 6af934acd83f (node segmeter ? (p_1 428.45)(e_1 2.36)))"})

#      The following is for a GET to the simple send me some data API:
#
#      response = ApiAdapter.send_command(
#        :resource => "/api_streams/1cff638538e73f/add_point?value=99.9",
#        :method => :get)
#
#      The following is for a GET to the work out if the lights are on at Inspire 9:
#
#      response = ApiAdapter.send_command(
#        :resource => "/api_streams/3f59386bcc7a2f7/is_on",
#        :method => :get)
#
#      Then bake your response any way you like!



require 'net/http'

LOG_SCREEN = false

module ApiAdapter

  def ApiAdapter.send_command(*args)

    # We aren't using args.extract_options! As this is simple ruby!

    resource = ""
    method = :get
    payload = nil

    # default host and port for the api

    host = "api.smartenergygroups.com"
    port = 80

    args.first.each do |key, value|
      case key
      when :resource
        resource = value
      when :payload
        payload = value
      when :method
        method = value
      when :host
        host = value
      when :port
        port = value
      end
    end


    # Open an HTTP connection to the server
    begin
      if LOG_SCREEN
        puts "The full resource to call: http://#{host.to_s}:#{port}#{resource}"
      end

      http = Net::HTTP.new(host, port)
      headers={}
      headers['Content-Type'] = "multipart/form-data"

      # Depending on the request type, create the appropriate objects.
      case method
      when :get
        # Append the payload to the URL
        if payload != nil
          resource << "?" + payload.map{|k,v| "#{k}=#{v}" }.join('&')
        end

        req = Net::HTTP::Get.new(resource)

      when :post
        # Set the form data with payload
        # command << "?" + options.map{|k,v| "#{k}=#{v}" }.join('&')
        req = Net::HTTP::Post.new(resource)
        req.set_form_data(payload)

      when :put
        # Set the form data with payload
        req = Net::HTTP::Put.new(resource)
        req.set_form_data(payload)
      end

      api_response = http.request(req)

      if api_response.is_a?(Net::HTTPSuccess)
        if LOG_SCREEN
          puts "the adaptor command to SEG API turned out just fine"
        end
      else
        if LOG_SCREEN
          puts "something went ka-boom with the SEG API adaptor command..."
        end
      end
      return api_response.body
    end
  rescue Exception => e
    puts "The internetz have pwned your rqst within this adaptrix Net::HTTP #{ e } (#{ e.class })!"
    return nil
  end
end

